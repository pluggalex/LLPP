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
	class waypoints;

	class Tagent_collection {
	public:
		Tagent_collection();

		void add_agent(int posX, int posY);

		// Returns the coordinates of the desired position
		std::vector<int> getDesiredX();
		std::vector<int> getDesiredY();

		// Sets the agent's position
		void setX(int newX);
		void setY(int newY);

		// Update the position according to get closer
		// to the current destination
		void computeNextDesiredPositionScalar(int start, int end);

		void computeNextDesiredPositionVector(int start, int end);

		// Position of agent defined by x and y
		// TODO
		//int getX() const { return x; };
		//int getY() const { return y; };

		// Adds a new waypoint to reach for this agent
		void addWaypoint(Twaypoint* wp);



	private:

		// The agent's current position
		std::vector<int> x;
		std::vector<int> y;

		// The agent's desired next position
		std::vector<int> desiredPositionX;
		std::vector<int> desiredPositionY;

		// The current destination (may require several steps to reach)
		waypoints* destinations; 



		// Returns the next destination to visit
		void getNextDestinationScalar(int index); //Should manage the Twaypoint collections above
	};
}

#endif