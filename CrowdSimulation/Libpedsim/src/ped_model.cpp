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

	// This is the sequential implementation
	implementation = PTHREAD;

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
	agentCollection->updateFromDesired(0, agentCollection->size());
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

// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.
/*
void Ped::Model::move(Ped::Tagent *agent)
{
	// Search for neighboring agents
	set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), 2);

	// Retrieve their positions
	std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
		takenPositions.push_back(position);
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<int, int> > prioritizedAlternatives;
	std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
	prioritizedAlternatives.push_back(pDesired);

	int diffX = pDesired.first - agent->getX();
	int diffY = pDesired.second - agent->getY();
	std::pair<int, int> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, agent->getY());
		p2 = std::make_pair(agent->getX(), pDesired.second);
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	// Find the first empty alternative position
	for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			// Set the agent's position 
			agent->setX((*it).first);
			agent->setY((*it).second);

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
set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist) const {

	// create the output list
	// ( It would be better to include only the agents close by, but this programmer is lazy.)	
	return set<const Ped::Tagent*>(agents.begin(), agents.end());
}

void Ped::Model::cleanup() {
	// Nothing to do here right now. 

}
*/

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
