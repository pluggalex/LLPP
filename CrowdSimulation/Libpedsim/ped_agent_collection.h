//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// TAgent represents an agent in the scenario. Each
// agent has a position (x,y) and a number of destinations
// it wants to visit (waypoints). The desired next position
// represents the position it would like to visit next as it
// will bring it closer to its destination.
// Note: the agent will not move by itself, but the movement
// is handled in ped_model.cpp. 
//

#ifndef _ped_agent_collection_h_
#define _ped_agent_collection_h_ 1

#include <vector>
#include <deque>

using namespace std;

namespace Ped {
	class Twaypoint;

	class Tagent_collection {
	public:
		Tagent_collection(int posX, int posY);
		Tagent_collection(double posX, double posY);

		// Returns the coordinates of the desired position
		std::vector<int> getDesiredX();
		std::vector<int> getDesiredY();

		// Sets the agent's position
		void setX(int newX);
		void setY(int newY);

		// Update the position according to get closer
		// to the current destination
		void computeNextDesiredPositionScalar(std::vector<int>::iterator start, 
											  std::vector<int>::iterator end);

		void computeNextDesiredPositionVector(std::vector<int>::iterator start,
											  std::vector<int>::iterator end);

		// Position of agent defined by x and y
		// TODO
		//int getX() const { return x; };
		//int getY() const { return y; };

		// Adds a new waypoint to reach for this agent
		void addWaypoint(Twaypoint* wp);



	private:
		Tagent_collection() {};

		// The agent's current position
		std::vector<int> x;
		std::vector<int> y;

		// The agent's desired next position
		std::vector<int> desiredPositionX;
		std::vector<int> desiredPositionY;

		// The current destination (may require several steps to reach)
		Twaypoint* destination; //Should be destination collection.
								//A collection of current destinations where
								//each element coresponds to the same element
								//in the X/Y vector above

		// The last destination
		//Twaypoint* lastDestination; <--IS THIS USED? //Same as above, put last instead of current

		// The queue of all destinations that this agent still has to visit
		deque<Twaypoint*> waypoints; //Let this be a collection of collections.
									 // Either by haveing each row be for one 'agent'
									 // or by having each row be for a collection 
									 // (each column will then be for each agent)

		// Internal init function 
		void init(int posX, int posY);

		// Returns the next destination to visit
		Twaypoint* getNextDestination(); //Should manage the Twaypoint collections above
	};
}

#endif