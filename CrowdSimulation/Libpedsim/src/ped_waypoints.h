//
// Adapted for Low Level Parallel Programming 2017
//
// Twaypoint represents a destination in a scenario.
// It is described by its position (x,y) and a radius
// which is used to define how close an agent has to be
// to this destination until we consider that the agent
// has reached its destination.int Ped::Twaypoint::staticid = 0;//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
#ifndef _ped_waypoints_h_
#define _ped_waypoints_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <cstddef>
#include <vector>
#include "ped_waypoint.h"

using namespace std;

namespace Ped {
	class Twaypoint;

	class LIBEXPORT waypoints {
	public:
		waypoints();
		virtual ~waypoints();

		// Sets the coordinates and the radius of this waypoint
		/*void setx(double px) { x = px; };
		void sety(double py) { y = py; };
		void setr(double pr) { r = pr; };

		int getid() const { return id; };
		*/
		void setAgents(int amount){ agents = amount; }

		std::vector<int>& getDestRefX() { return waypointX.front(); };
		std::vector<int>& getDestRefY() { return waypointY.front(); };
		std::vector<int>& getDestRefR() { return waypointR.front(); };
		
		std::vector<std::vector<int>> getX() const { return waypointX; }
		std::vector<std::vector<int>> getY() const { return waypointY; }
		std::vector<std::vector<int>> getR() const { return waypointR; }

		//Range of agents
		void addWaypoint(Ped::Twaypoint* waypoint, int start, int end);

		//Add waypoint for single agent
		//void addWaypoint(Ped::Twaypoint* waypoint, int start, int end);

		//Sends the first waypoint for the given agent to the back of the queue
		//by swaping each element on the way, effectivly 'bubbling' down the waypoint
		void rotateQueue(int agent);

		

		waypoints& operator+=(const Ped::waypoints& rhs);

	private:
		
		int agents;

		std::vector<std::vector<int>> waypointX;
		std::vector<std::vector<int>> waypointY;
		std::vector<std::vector<int>> waypointR;
	};
}

#endif
