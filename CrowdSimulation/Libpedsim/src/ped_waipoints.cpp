//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_waypoints.h"

#include <cmath>

// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

// initialize static variables
//int Ped::waypoints::staticid = 0;

// Constructor - sets the most basic parameters.
Ped::waypoints::waypoints() : agents(0), waypointX(0), waypointY(0), waypointR(0) {};

Ped::waypoints::~waypoints() {};

void Ped::waypoints::addWaypoint(Ped::Twaypoint* waypoint, int start, int end){
	int size = waypointX.size();
	//Create space for another waypoint in the vectors
	waypointX.push_back(std::vector<int>(agents, NULL));
	waypointY.push_back(std::vector<int>(agents, NULL));
	waypointR.push_back(std::vector<int>(agents, NULL));

	//Fille the empty waypoint space with coordinates
	std::fill(waypointX[size - 1].begin() + start, waypointX[size - 1].begin() + end, waypoint->getx());
	std::fill(waypointY[size - 1].begin() + start, waypointY[size - 1].begin() + end, waypoint->gety());
	std::fill(waypointR[size - 1].begin() + start, waypointR[size - 1].begin() + end, waypoint->getr());
}


