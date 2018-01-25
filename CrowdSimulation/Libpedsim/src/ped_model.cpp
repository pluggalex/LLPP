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

void Ped::Model::setup(std::vector<Ped::Tagent*> agentsInScenario, std::vector<Twaypoint*> destinationsInScenario)
{
	// Convenience test: does CUDA work on this machine?
	cuda_test();

	// Set 
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());

	// Set up destinations
	destinations = std::vector<Ped::Twaypoint*>(destinationsInScenario.begin(), destinationsInScenario.end());

	// This is the sequential implementation
	implementation = PTHREAD_2;

	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();

	threadNum = 4;
	if (implementation == OMP_2)
		omp_set_num_threads(threadNum);

	//Split up the workload in chunks of N number of threads
	chunkUp();

	// Prep the threads for parallelization
	if (implementation == PTHREAD_2)
		Ped::Model::pthreadPrepTick();
}


// Sequential tick..
void Ped::Model::seqTick(){
	for (auto agent : this->agents){
		agent->computeNextDesiredPosition();
		agent->setX(agent->getDesiredX());
		agent->setY(agent->getDesiredY());
	}
}


/*
*	OpenMP version of tick().
*	Simpler version where the workload is chunked up by OpenMP
*/
void Ped::Model::ompTick(){
	int size = this->agents.size();

	#pragma omp parallel for 
	for (int i = 0; i < size; i++){
		Tagent *agent = this->agents[i];
		agent->computeNextDesiredPosition();
		agent->setX(agent->getDesiredX());
		agent->setY(agent->getDesiredY());
	}
}

/*
*	OpenMP version of tic()
*	Chunks are already divided and the work to be done
*   can be done in parallel.
*/
void Ped::Model::ompTick_ver2(){
	#pragma omp parallel
	{
		int thread_id = omp_get_thread_num();

		for (int j = (*chunks)[thread_id] * thread_id; j < (*chunks)[thread_id] * (thread_id + 1); j++)
		{
			Tagent *agent = this->agents[j];
			agent->computeNextDesiredPosition();
			agent->setX(agent->getDesiredX());
			agent->setY(agent->getDesiredY());
		}
	}	
}

/* old thread version*/

void Ped::Model::computeAgentsInRange(std::vector<Tagent*>::iterator current, std::vector<Tagent*>::iterator end){
	for (current; current < end; current++){
		(*current)->computeNextDesiredPosition();
		(*current)->setX((*current)->getDesiredX());
		(*current)->setY((*current)->getDesiredY());
	}
}

void Ped::Model::pthreadTick(){
	tickThreads = new std::thread[threadNum];
	int totalSize = this->agents.size();
	int chunkSize = totalSize / threadNum;
	int rest = totalSize % threadNum;

	std::vector<int> chunks( threadNum, chunkSize );
	for (int i = 0; i < rest; i++){
		chunks[i] += 1;
	}

	std::vector<Tagent*>::iterator agentsBegin = this->agents.begin();
	std::vector<Tagent*>::iterator start;
	std::vector<Tagent*>::iterator end;
	for (int t = 0; t < threadNum; t++){
		start = agentsBegin + (chunks[t] * t);
		end = agentsBegin + (chunks[t] * (t + 1));
		tickThreads[t] = std::thread(&Ped::Model::computeAgentsInRange, this, start, end);
	}
}



/* 
*	Thread version of the tick functions compute part.
*	Computes and Sets the destination of all agents in the given range.
*/
void Ped::Model::computeAgentsInRange_version2(std::vector<Tagent*>::iterator start, std::vector<Tagent*>::iterator end, int threadId){
	while (threadCompActive){

		// Wait for the go signal from tick()
		WaitForSingleObject(agentComputationSem[threadId], INFINITE);
		
		std::vector<Tagent*>::iterator current = start;
		for (current; current < end; current++){
			(*current)->computeNextDesiredPosition();
			(*current)->setX((*current)->getDesiredX());
			(*current)->setY((*current)->getDesiredY());
		}
		
		/* 
		*	Instead of joining() on each thread we tell the tick() that
		*	we are done by semaphore.
		*/
		ReleaseSemaphore(agentCompletionSem[threadId], 1, NULL);
	}
}

void Ped::Model::pthreadTick_ver2(){
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
	std::vector<Tagent*>::iterator agentsBegin = this->agents.begin();
	std::vector<Tagent*>::iterator start;
	std::vector<Tagent*>::iterator end;
	for (int tId = 0; tId < threadNum; tId++){
		start = agentsBegin + (*chunks)[tId] * tId;
		end = start + (*chunks)[tId];
		tickThreads[tId] = std::thread(&Ped::Model::computeAgentsInRange_version2, this, start, end, tId);
	}
}

// Splits up the workload on N number of chunks where N is the number of threads in use
void Ped::Model::chunkUp(){
	int totalSize = this->agents.size();
	int chunkSize = totalSize / threadNum;
	int rest = totalSize % threadNum;

	chunks = new std::vector<int>(threadNum, chunkSize);
	for (int i = 0; i < rest; i++){
		(*chunks)[i] += 1;
	}
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

	case OMP_2:
		this->ompTick_ver2();
		break;

	case PTHREAD_2:	
		this->pthreadTick_ver2();
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

void Ped::Model::killThreads(){
	threadCompActive = false;
	pthreadTick_ver2();
	for (int t = 0; t < threadNum; t++){
		tickThreads[t].join();
	}

	delete[] tickThreads;
}

Ped::Model::~Model()
{
	if (implementation == PTHREAD_2)
		killThreads();

	std::for_each(agents.begin(), agents.end(), [](Ped::Tagent *agent){delete agent; });
	std::for_each(destinations.begin(), destinations.end(), [](Ped::Twaypoint *destination){delete destination; });

	std::for_each(agentComputationSem.begin(), agentComputationSem.end(), [](HANDLE sem){CloseHandle(sem); });
	std::for_each(agentCompletionSem.begin(), agentCompletionSem.end(), [](HANDLE sem){CloseHandle(sem); });
	
	delete chunks;
}
