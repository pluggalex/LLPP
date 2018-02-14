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

	std::vector<float>* Xptr = agentCollection->getXptr();
	std::vector<float>* Yptr = agentCollection->getYptr();
	std::vector<float>* XDesptr = agentCollection->getDesiredXptr();
	std::vector<float>* YDesptr = agentCollection->getDesiredYptr();

	for (int i = 0; i < (*Xptr).size(); i++){
		QT_X.push_back(&((*Xptr)[i]));
		QT_Y.push_back(&((*Yptr)[i]));
		QT_DesX.push_back(&((*XDesptr)[i]));
		QT_DesY.push_back(&((*YDesptr)[i]));
	}

	// This is the sequential implementation
	implementation = SEQ;

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
	move(/*Region*/);
	//agentCollection->updateFromDesired(0, agentCollection->size());
}


void Ped::Model::ompTick(){
	#pragma omp parallel
	{
		int thread_id = omp_get_thread_num();
		int start = (*chunks)[thread_id] * thread_id;
		int end = (*chunks)[thread_id] * (thread_id + 1);

		agentCollection->computeNextDesiredPositionScalar(start, end);
		agentCollection->updateFromDesired(start, end);
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
	// EDIT HERE FOR ASSIGNMENT 1

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


}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////


void Ped::Model::move(std::vector<float*>& regionX, std::vector<float*>& regionY, 
					  std::vector<float*>& regionDesX, std::vector<float*>& regionDesY){
	for (int i = 0; i < regionX.size(); i++){
		collisionHandler(i, regionX, regionY, regionDesX, regionDesY);
	}
}



void Ped::Model::collisionHandler(int index,
								  std::vector<float*>& regionX,
								  std::vector<float*>& regionY,
								  std::vector<float*>& regionDesX,
								  std::vector<float*>& regionDesY){

	// Search for neighboring agents
	int distanceToNeighbor = 2;
	std::vector<vector<float*>> neighbors = getNeighbors(regionX, regionY, index, distanceToNeighbor);//Getting all x and y's atm
	std::vector<float*> neighborX = neighbors[0];
	std::vector<float*> neighborY = neighbors[1];

	// Retrieve their positions
	std::vector<std::pair<float, float> > takenPositions;
	for (int i = 0; i < neighborX.size(); i++) {
		std::pair<float, float> position((*neighborX[i]), (*neighborY[i]));
		takenPositions.push_back(position);

	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<float, float> > prioritizedAlternatives;
	std::pair<float, float> pDesired((*regionDesX[index]), (*regionDesY[index]));
	prioritizedAlternatives.push_back(pDesired);

	int diffX = pDesired.first - (*regionX[index]);
	int diffY = pDesired.second - (*regionY[index]);
	std::pair<float, float> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, (*regionY[index]));
		p2 = std::make_pair((*regionX[index]), pDesired.second);
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	// Find the first empty alternative position
	for (std::vector<pair<float, float> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			// Set the agent's position 
			//(*agentDesiredXs)[index] = (*it).first;
			//(*agentDesiredYs)[index] = (*it).second;

			(*regionX[index]) = (*it).first;
			(*regionY[index]) = (*it).second;

			break;
		}
	}
}






// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.

void Ped::Model::move(int start, int end)
{

	std::vector<float>* agentXs = this->agentCollection->getXptr();
	std::vector<float>* agentYs = this->agentCollection->getYptr();

	std::vector<float>* agentDesiredXs = this->agentCollection->getDesiredXptr();
	std::vector<float>* agentDesiredYs = this->agentCollection->getDesiredYptr();

	for (int i = start; i < end; i++){
		collisionHandler(i, agentXs, agentYs, agentDesiredXs, agentDesiredYs);
	}
}

void Ped::Model::collisionHandler(int index, 
									std::vector<float>* agentXs, 
									std::vector<float>* agentYs,	
									std::vector<float>* agentDesiredXs, 
									std::vector<float>* agentDesiredYs){
	// Search for neighboring agents
	std::vector<vector<float>> neighbors = getNeighbors(1,2,3);//Getting all x and y's atm
	std::vector<float> neighborX = neighbors[0];
	std::vector<float> neighborY = neighbors[1];
	std::vector<float> neighborDesiredX = neighbors[0];
	std::vector<float> neighborDesiredY = neighbors[0];

	// Retrieve their positions
	std::vector<std::pair<float, float> > takenPositions;
	for (int i = 0; i < neighborX.size(); i++) {
		std::pair<float, float> position(neighborX[i], neighborY[i]);
		takenPositions.push_back(position);
		//std::pair<float, float> position2(neighborDesiredX[i], neighborDesiredY[i]);
		//takenPositions.push_back(position2);
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<float, float> > prioritizedAlternatives;
	std::pair<float, float> pDesired((*agentDesiredXs)[index], (*agentDesiredYs)[index]);
	prioritizedAlternatives.push_back(pDesired);

	int diffX = pDesired.first - (*agentXs)[index];
	int diffY = pDesired.second - (*agentYs)[index];
	std::pair<float, float> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, (*agentYs)[index]);
		p2 = std::make_pair((*agentXs)[index], pDesired.second);
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	// Find the first empty alternative position
	for (std::vector<pair<float, float> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			// Set the agent's position 
			//(*agentDesiredXs)[index] = (*it).first;
			//(*agentDesiredYs)[index] = (*it).second;

			(*agentXs)[index] = (*it).first;
			(*agentYs)[index] = (*it).second;

			break;
		}
	}
}


/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
std::vector<vector<float>> Ped::Model::getNeighbors(int x = 1, int y = 1, int dist = 1) const {

	//std::vector<float>* agentXs = this->agentCollection->getXptr();
	//std::vector<float>* agentYs = this->agentCollection->getYptr();
	//int xTop = x ;


	// create the output list
	// ( It would be better to include only the agents close by, but this programmer is lazy.)	
	std::vector<std::vector<float>> res;
	res.push_back(this->agentCollection->getX());
	res.push_back(this->agentCollection->getY());
	res.push_back(this->agentCollection->getDesiredX());
	res.push_back(this->agentCollection->getDesiredY());
	return res;
}


std::vector<vector<float*>> Ped::Model::getNeighbors(std::vector<float*> &regionX,
													std::vector<float*> &regionY,
													int index,
													int dist) const {

	//std::vector<float>* agentXs = this->agentCollection->getXptr();
	//std::vector<float>* agentYs = this->agentCollection->getYptr();
	//int xTop = x ;


	// create the output list
	// ( It would be better to include only the agents close by, but this programmer is lazy.)	
	std::vector<std::vector<float*>> res;
	res.push_back(regionX);
	res.push_back(regionY);
	return res;
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
}
