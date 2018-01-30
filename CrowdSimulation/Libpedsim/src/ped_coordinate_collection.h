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
#ifndef _ped_coordinate_collection_h_
#define _ped_coordinate_collection_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <cstddef>
#include <vector>

using namespace std;

namespace Ped {
	class LIBEXPORT coordinate_collection {
	public:
		coordinate_collection();
		coordinate_collection(double x, double y, double r);
		virtual ~coordinate_collection();

		// Sets the coordinates and the radius of this waypoint
		/*void setx(double px) { x = px; };
		void sety(double py) { y = py; };
		void setr(double pr) { r = pr; };

		int getid() const { return id; };
		*/
		std::vector<int>* getx() const { return x; };
		std::vector<int>* gety() const { return y; };
		std::vector<int>* getr() const { return r; };
		
	protected:
		// id incrementer, used for assigning unique ids
		//static int staticid;

		// waypoint id
		//int id;

		// position of the waypoint
		std::vector<int>* x;
		std::vector<int>* y;

		// radius defines the area of this waypoint, i.e. a circular
		// area with the middle point in (x,y).
		// Any point within this area is considered to be belonging
		// to this destination.
		std::vector<int>* r;
	};
}

#endif
