//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent_collection.h"
#include "ped_waypoints.h"
#include "ped_waypoint.h"
#include <math.h>
#include <algorithm>

// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

Ped::Tagent_collection::Tagent_collection() {destinations = new Ped::waypoints(); }

void Ped::Tagent_collection::add_agent(int posX, int posY){
	x.push_back(posX);
	y.push_back(posY);
}


// TODO 
// Setters and getters
// -----------------------------------
void Ped::Tagent_collection::setX(int newX){}
void Ped::Tagent_collection::setY(int newY){}

std::vector<int> Ped::Tagent_collection::getDesiredX(){}
std::vector<int> Ped::Tagent_collection::getDesiredY(){}
//------------------------------------



void Ped::Tagent_collection::computeNextDesiredPositionScalar(int start, int end) {


	for (int index = start; index < end; index++){
		getNextDestinationScalar(index);

		if (!(*destination->getx())[0].empty() && (*destination->getx())[0][index] != NULL) {
			double diffX = (*destination->getx())[index] - x[index];
			double diffY = (*destination->gety())[index] - y[index];
			double len = sqrt(diffX * diffX + diffY * diffY);
			desiredPositionX[index] = (int)round(x[index] + diffX / len);
			desiredPositionY[index] = (int)round(y[index] + diffY / len);
	}
}

void Ped::Tagent_collection::addWaypoint(Twaypoint* wp, int start_agent, int end_agent) {
	destinations->addWaypoint(wp, );

}

Ped::Twaypoint* Ped::Tagent_collection::getNextDestinationScalar(int index) {
	Ped::Twaypoint* nextDestination = NULL;
	bool agentReachedDestination = false;

	if (destination->x[index] != NULL) {
		// compute if agent reached its current destination
		double diffX = *(destination->getx())[index] - x[index];
		double diffY = *(destination->gety())[index] - y[index];
		double length = sqrt(diffX * diffX + diffY * diffY);
		agentReachedDestination = length < *(destination->getr())[index];
	}

	if ((agentReachedDestination || destination == NULL) && !waypoints.empty()) {
		// Case 1: agent has reached destination (or has no current destination);
		// get next destination if available
		waypoints.push_back(destination);
		nextDestination = waypoints.front();
		waypoints.pop_front();
	}
	else {
		// Case 2: agent has not yet reached destination, continue to move towards
		// current destination
		nextDestination = destination;
	}

	return nextDestination;
}
