//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// Model coordinates a time step in a scenario: for each
// time step all agents need to be moved by one position if
// possible.
//
#ifndef _ped_model_h_
#define _ped_model_h_

#include <vector>
#include <map>
#include <set>
#include <Windows.h>
#include <thread>
#include <memory>

#include "ped_agent_collection.h"

namespace Ped{
	class Tagent_collection;

	// The implementation modes for Assignment 1 + 2:
	// chooses which implementation to use for tick()
	enum IMPLEMENTATION { CUDA, VECTOR, OMP, PTHREAD, SEQ, OMP_2, PTHREAD_2};

	class Model
	{
	public:

		// Sets everything up
		void setup(std::unique_ptr<Tagent_collection> agent_collection);

		// Coordinates a time step in the scenario: move all agents by one step (if applicable).
		void tick();

		// Sequential version of tick()
		void seqTick();

		// OpenMP version of tick()
		//void ompTick();

		// OpenMP version_2 of tick()
		void ompTick_ver2();

		// Pthread version of tick()
		//void pthreadTick();
		
		void pthreadTick_ver2();

		void pthreadPrepTick();

		void killThreads();

		// Helper to split up the workload in chunks
		void chunkUp();

		// Go from iterator start to end and set X and Y for the agents in that range
		//void computeAgentsInRange(std::vector<Tagent*>::iterator, std::vector<Tagent*>::iterator);


		void computeAgentsInRange_version2(int start, int end, int tId);
		
		// Returns the agents of this scenario
		//const std::vector<Tagent*> getAgents() const { return agents; };

		// Adds an agent to the tree structure
		//void placeAgent(const Ped::Tagent *a);

		// Cleans up the tree and restructures it. Worth calling every now and then.
		void cleanup();
		~Model();

		// Returns the heatmap visualizing the density of agents
		int const * const * getHeatmap() const { return blurred_heatmap; };
		int getHeatmapSize() const;


	private:

		// Denotes which implementation (sequential, parallel implementations..)
		// should be used for calculating the desired positions of
		// agents (Assignment 1)
		IMPLEMENTATION implementation;

		// The agents in this scenario
		std::unique_ptr<Tagent_collection> agentCollection;

		// The waypoints in this scenario
		std::vector<Twaypoint*> destinations;

		std::vector<HANDLE> agentComputationSem;

		std::vector<HANDLE> agentCompletionSem;

		bool threadCompActive;

		std::thread* tickThreads;

		int threadNum;

		//chunks containing the workload for each thread
		std::vector<int> *chunks;

		// Moves an agent towards its next position
		//void move(Ped::Tagent *agent);

		
		////////////
		/// Everything below here won't be relevant until Assignment 3
		///////////////////////////////////////////////

		// Returns the set of neighboring agents for the specified position
		//set<const Ped::Tagent*> getNeighbors(int x, int y, int dist) const;

		////////////
		/// Everything below here won't be relevant until Assignment 4
		///////////////////////////////////////////////

#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE SIZE*CELLSIZE

		// The heatmap representing the density of agents
		int ** heatmap;

		// The scaled heatmap that fits to the view
		int ** scaled_heatmap;

		// The final heatmap: blurred and scaled to fit the view
		int ** blurred_heatmap;

		void setupHeatmapSeq();
		void updateHeatmapSeq();
	};
}
#endif
