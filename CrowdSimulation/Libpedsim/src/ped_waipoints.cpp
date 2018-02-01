//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_waypoints.h"
#include <algorithm>

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


// We will begin to assume the given range is covering all the agents..
void Ped::waypoints::addWaypoint(Ped::Twaypoint* waypoint, int start, int end){
	
	//Create space for another waypoint in the vectors
	waypointX.push_back(std::vector<int>(agents, NULL));
	waypointY.push_back(std::vector<int>(agents, NULL));
	waypointR.push_back(std::vector<int>(agents, NULL));

	int size = waypointX.size() - 1;
	//Fille the empty waypoint space with coordinates
	std::fill(waypointX[size].begin() + start, waypointX[size].begin() + end, waypoint->getx());
	std::fill(waypointY[size].begin() + start, waypointY[size].begin() + end, waypoint->gety());
	std::fill(waypointR[size].begin() + start, waypointR[size].begin() + end, waypoint->getr());
}

// TODO
// Make sure that the nullvalues gets handled somehow
void bubbleToBack(int agent, std::vector<std::vector<int>>& waypointQueue){
	for (int i = 0; i < waypointQueue.size()-1; i++){
		std::swap(waypointQueue[i].begin() + agent, waypointQueue[i+1].begin() + agent);
	}
}

//Takes an index representing an agent and sends the
//first waypoint for that agent to the back
void Ped::waypoints::rotateQueue(int agent){
	bubbleToBack(agent, waypointX);
	bubbleToBack(agent, waypointY);
	bubbleToBack(agent, waypointR);
}