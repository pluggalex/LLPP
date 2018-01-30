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
		std::vector<std::vector<int>>& getx() { return waypointX; };
		std::vector<std::vector<int>>& gety() { return waypointY; };
		std::vector<std::vector<int>>& getr() { return waypointR; };
		
		//Range of agents
		void addWaypoint(Ped::Twaypoint* waypoint, int start, int end);

		//Add waypoint for single agent
		void addWaypoint(Ped::Twaypoint* waypoint, int start, int end);

		void rollQueue(int index);

	private:
		
		int agents;

		std::vector<std::vector<int>> waypointX;
		std::vector<std::vector<int>> waypointY;
		std::vector<std::vector<int>> waypointR;
	};
}

#endif
