//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_model.h"
#include "ped_waypoint.h"
#include "ped_model.h"
#include <iostream>
#include <stack>
#include <algorithm>
#include "cuda_testkernel.h"
#include <omp.h>


// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

void Ped::Model::setup(std::unique_ptr<Tagent_collection> _agentCollection)
{
	// Convenience test: does CUDA work on this machine?
	cuda_test();

	// Set 
	agentCollection = std::move(_agentCollection);

	// Set up destinations
	//destinations = std::vector<Ped::Twaypoint*>(destinationsInScenario.begin(), destinationsInScenario.end());

	

	int xLeft = 0;
	int xRight = 800;
	int yTop = 0;
	int yBot = 600;
	int treeLevel = 1;
	/*All the child trees will be deleted within the root node. Promise. Hopefully.*/
	QTree* childBotLeft = new QTree({ xLeft, yBot / 2 }, { xRight / 2, yBot }, treeLevel, agentCollection->size());
	QTree* childBotRight = new QTree({ xRight / 2, yBot / 2 }, { xRight, yBot }, treeLevel, agentCollection->size());
	QTree* childTopLeft = new QTree({ xLeft, yTop }, { xRight / 2, yBot / 2 }, treeLevel, agentCollection->size());
	QTree* childTopRight = new QTree({ xRight / 2, yTop }, { xRight, yBot / 2 }, treeLevel, agentCollection->size());

	//The root tree. Tree level will automaticly be 0
	rootRegion = new QTree({ xLeft, yTop }, { xRight, yBot }, 
							childBotLeft, childBotRight, childTopLeft, childTopRight, agentCollection->size());


	/*Get the data we want to distribute over our regions*/
	std::vector<float>* Xptr = agentCollection->getXptr();
	std::vector<float>* Yptr = agentCollection->getYptr();
	std::vector<float>* XDesptr = agentCollection->getDesiredXptr();
	std::vector<float>* YDesptr = agentCollection->getDesiredYptr();
	
	//Temps
	std::vector<std::vector<float*>> tempBuff;
	int size = Xptr->size();
	
	//Copy pointers to our data to the regions
	for (int i = 0; i < size; i++){
		tempBuff.emplace_back(std::initializer_list<float*>{&((*Xptr)[i]), &((*Yptr)[i]), &((*XDesptr)[i]), &((*YDesptr)[i])});
	}
	for (auto& agent : tempBuff){
		rootRegion->insert(agent, (*agent[agentIndex::X]), (*agent[agentIndex::Y]));
	}

	rootRegion->flushAllBuffers();
	rootRegion->growAllTrees();

	// This is the sequential implementation
	implementation = OMP;


	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();

	threadNum = 2;
	if (implementation == OMP)
		omp_set_num_threads(threadNum);

	//Split up the workload in chunks of N number of threads
	chunkUp();

	// Prep the threads for parallelization
	if (implementation == PTHREAD)
		Ped::Model::pthreadPrepTick();
}


// Sequential tick..
void Ped::Model::seqTick(){
	agentCollection->computeNextDesiredPositionScalar(0, agentCollection->size());
	std::vector<QTree*> leafNodes = rootRegion->getLeafNodes();
	for (auto leaf : leafNodes)
		move(leaf);
	//agentCollection->updateFromDesired(0, agentCollection->size());
}


void Ped::Model::ompTick(){
	#pragma omp parallel
	{
		int thread_id = omp_get_thread_num();
		int prevId = 0;
		int start = 0;// (*chunks)[thread_id] * thread_id;

		if (thread_id - 1 >= 0){
			prevId = thread_id - 1;
			start = (*chunks)[prevId];
		}
		int end = start + (*chunks)[thread_id];

		agentCollection->computeNextDesiredPositionScalar(start, end);
		
		#pragma omp barrier
		
		
		start = 0;
		if (thread_id - 1 >= 0){
			prevId = thread_id - 1;
			start = (*leafChunks)[prevId];
		}
		end = start + (*leafChunks)[thread_id];
		for (int chunk = start; chunk < end; chunk++)
			move(leafNodes[chunk]);
		//agentCollection->updateFromDesired(start, end);
	}	
}


/* 
*	Thread version of the tick functions compute part.
*	Computes and Sets the destination of all agents in the given range.
*/
void Ped::Model::computeAgentsInRange_version2(int start, int end, int threadId){
	while (threadCompActive){
		// Wait for the go signal from tick()
		WaitForSingleObject(agentComputationSem[threadId], INFINITE);
		
		//Do the work
		agentCollection->computeNextDesiredPositionVector(start, end);
		agentCollection->updateFromDesired(start, end);

		/* 
		*	Instead of joining() on each thread we tell the tick() that
		*	we are done by semaphore.
		*/
		ReleaseSemaphore(agentCompletionSem[threadId], 1, NULL);
	}
}

void Ped::Model::pthreadTick(){
	for (int i = 0; i < agentComputationSem.size(); i++){
		ReleaseSemaphore(agentComputationSem[i], 1, NULL);
	}

	WaitForMultipleObjects(threadNum, &(agentCompletionSem[0]), TRUE, INFINITE);
}


void Ped::Model::pthreadPrepTick(){

	// Create the semaphores which will control the threads
	agentComputationSem.reserve(threadNum);
	agentCompletionSem.reserve(threadNum);
	for (int i = 0; i < threadNum; i++){
		agentComputationSem.emplace_back(CreateSemaphore(NULL, 0, 1, NULL));
		agentCompletionSem.emplace_back(CreateSemaphore(NULL, 0, 1, NULL));
	}

	threadCompActive = true; // Set to false when we want to stop the threads
	tickThreads = new std::thread[threadNum];

	// Give each thread a chunk to work on, then just start the threads.
	// They will wait for the semaphores before doing the computation
	int start, end;
	for (int tId = 0; tId < threadNum; tId++){
		start = (*chunks)[tId] * tId;
		end = start + (*chunks)[tId];
		tickThreads[tId] = std::thread(&Ped::Model::computeAgentsInRange_version2, this, start, end, tId);
	}
}

void Ped::Model::chunkUpRegions(){
	leafNodes = this->rootRegion->getLeafNodes();
	int totalSize = leafNodes.size();
	int chunkSize = totalSize / threadNum;
	int rest = totalSize % threadNum;

	leafChunks = new std::vector<int>(threadNum, chunkSize);
	for (int i = 0; i < rest; i++){
		(*leafChunks)[i]++;
	}
}

// Splits up the workload on N number of chunks where N is the number of threads in use
void Ped::Model::chunkUp(){
	int totalSize = this->agentCollection->size();
	int registerSize = 4;
	int numRegisters = totalSize / registerSize;
	int restElems = totalSize % registerSize;

	int chunkSize = (numRegisters / threadNum) * registerSize;
	int restChunks = numRegisters % threadNum;

	chunks = new std::vector<int>(threadNum, chunkSize);
	for (int i = 0; i < restChunks; i++){
		(*chunks)[i] += registerSize;
	}

	if (restElems)
		(*chunks)[threadNum - 1] += registerSize;
}

void Ped::Model::tick()
{
	//std::cout << "-------------------------------------------------\n"
	//		  << "                 START TICK\n"
	//		  << "-------------------------------------------------\n";

	
	chunkUpRegions();

	switch (this->implementation){
	case SEQ:
		this->seqTick();
		break;

	case OMP:
		this->ompTick();
		break;

	case PTHREAD: 
		this->pthreadTick();
		break;

	default:
		break;
	}

	rootRegion->flushAllBuffers();
	rootRegion->purgeAllRegions();
	rootRegion->growAllTrees();
	/*std::vector<QTree*>leafs = rootRegion->getLeafNodes();
	for (auto& leaf : leafs)
		leaf->printCorners();
	std::cout << "-------------------------------------------------\n"
		      << "                 END TICK\n"
			  << "-------------------------------------------------\n";
			  */
}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////


void Ped::Model::move(QTree* region){
	for (int i = 0; i < region->getXs().size(); i++){
		collisionHandler(i, region);
	}
}

std::vector<std::pair<float, float>> calcNeighbors(float x, float y, int minX, int minY, int maxX, int maxY){
	std::vector<std::pair<float, float>> neighbors;
	float nX, nY;
	for (float i = -1; i < 2; i++){
		for (float j = -1; j < 2; j++){
			nY = i + y;
			nX = j + x;
			if (nX >= minX && nX < maxX && nY >= minY && nY < maxY && !(nX == x && nY == y)){
				neighbors.push_back(std::pair<float, float>(nX, nY));
			}
			if (nX == 0 && nY == 1)
				std::cout << "shit" << " (" << x << "," << y << ")";
		}
	}
	return neighbors;
}

void Ped::Model::collisionHandler(int index, QTree* region){

	std::vector<float*> regionXs = region->getXs();
	std::vector<float*> regionYs = region->getYs();
	float* currentX = regionXs[index];
	float* currentY = regionYs[index];
	std::pair<int, int> topLeft = region->getTopLeft();
	std::pair<int, int> botRight = region->getBotRight();
	std::vector<std::pair<float, float>> neighbors = calcNeighbors(*currentX, *currentY,
																   topLeft.first, topLeft.second,
																   botRight.first, botRight.second);

	// Retrieve their positions
	std::vector<std::pair<float, float> > takenPositions;

	for (int i = 0; i < regionXs.size(); i++){
		for (auto neighbor : neighbors){
			if (*(regionXs[i]) == neighbor.first && *(regionYs[i]) == neighbor.second)
				takenPositions.push_back(neighbor);
		}
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<float, float> > prioritizedAlternatives;
	std::pair<float, float> pDesired((*region->getDesiredXs()[index]), (*region->getDesiredYs()[index]));
	prioritizedAlternatives.push_back(pDesired);

	/*if (pDesired.first == *currentX && pDesired.second == *currentY){
		std::cout << "ojoj!";
	}
	*/

	int desiredIndex = -1;
	for (int i = 0; i < neighbors.size(); i++){
		if (neighbors[i].first == pDesired.first  && neighbors[i].second == pDesired.second){
			desiredIndex = i;
			break;
		}
	}

	if (desiredIndex != -1){
		int leftIndex, rightIndex;
		for (int i = 1; i <= (neighbors.size() - 1) / 2; i++){
			leftIndex = desiredIndex - i;
			if (leftIndex < 0)
				leftIndex = neighbors.size() + leftIndex;

			rightIndex = desiredIndex + i;
			if (rightIndex > neighbors.size() - 1)
				rightIndex = rightIndex - neighbors.size();

			prioritizedAlternatives.push_back(neighbors[leftIndex]);
			prioritizedAlternatives.push_back(neighbors[rightIndex]);

		}
	}

	if (neighbors.size() > 0 && (neighbors.size() - 1) % 2){
		int i = ((neighbors.size() - 1) / 2) + desiredIndex + 1;
		if (i > neighbors.size() - 1)
			i = i - neighbors.size();
		prioritizedAlternatives.push_back(neighbors[i]);
	}

	// Find the first empty alternative position
	for (std::vector<pair<float, float> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {
			float desiredX = (*it).first;
			float desiredY = (*it).second;
			float* x = region->getXs()[index];
			float* y = region->getYs()[index];
			float* prioDesX = region->getDesiredXs()[index];
			float* prioDesY = region->getDesiredYs()[index];

			if (!(region->inBounds(desiredX, desiredY))){// && rootRegion->isCoordFree(desiredX, desiredY)){
				region->remove(*x, *y, index);
				rootRegion->insert(std::vector<float*>{x, y, prioDesX, prioDesY}, std::vector<std::pair<float, float>>(it, prioritizedAlternatives.end()));
				break;
			}
			else if (region->inBounds(desiredX, desiredY)){
				(*x) = desiredX;
				(*y) = desiredY;
				break;
			}
		}
	}
}


void Ped::Model::cleanup() {
	// Nothing to do here right now. 

}


// Deletes the thread in a sort of safe manner
void Ped::Model::killThreads(){
	threadCompActive = false; // Will make the tread finish return once another tick is done
	pthreadTick();		  // Call for a last tick

	// Join up and delete the threads
	for (int t = 0; t < threadNum; t++){
		tickThreads[t].join();//
	}

	delete[] tickThreads;
}

Ped::Model::~Model()
{
	if (implementation == PTHREAD_2)
		killThreads();

	//std::for_each(agents.begin(), agents.end(), [](Ped::Tagent *agent){delete agent; });
	//std::for_each(destinations.begin(), destinations.end(), [](Ped::Twaypoint *destination){delete destination; });

	std::for_each(agentComputationSem.begin(), agentComputationSem.end(), [](HANDLE sem){CloseHandle(sem); });
	std::for_each(agentCompletionSem.begin(), agentCompletionSem.end(), [](HANDLE sem){CloseHandle(sem); });
	
	delete chunks;
	delete leafChunks;
	delete rootRegion;
}
