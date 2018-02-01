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
#include <memory>
//#include <utility>
#include "ped_waypoints.h"
#include "ped_waypoint.h"

using namespace std;

namespace Ped {
	class Twaypoint;

	class Tagent_collection {
	public:
		Tagent_collection();

		// Returns the coordinates of the desired position
		//std::vector<int> getDesiredX();
		//std::vector<int> getDesiredY();

		int size() { return x.size(); }

		// Sets the agent's position
		//void setX(int newX);
		//void setY(int newY);

		void updateFromDesired(int start_agent, int end_agent);

		// Update the position according to get closer
		// to the current destination
		void computeNextDesiredPositionScalar(int start, int end);

		//void computeNextDesiredPositionVector(int start, int end);

		// Position of agent defined by x and y
		// TODO
		std::vector<int> getX() const { return x; };
		std::vector<int> getY() const { return y; };
		Ped::waypoints* borrowDestinations() const { return destinations.get(); }

		// Add a new waypoint for every agent
		void addWaypoint(Twaypoint* wp);

		void addAgent(int x, int y);

		Ped::Tagent_collection operator+=(const Ped::Tagent_collection& rhs);


	private:
		// The agent's current position
		std::vector<int> x;
		std::vector<int> y;

		// The agent's desired next position
		std::vector<int> desiredPositionX;
		std::vector<int> desiredPositionY;

		// The current destination (may require several steps to reach)
		std::unique_ptr<Ped::waypoints> destinations; //This is a combined destination and queue atm

		// Returns the next destination to visit
		void setNextDestinationScalar(int start_agent, int end_agent);
	};
}

#endif