//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent_collection.h"
#include "ped_waypoint.h"
#include <math.h>

// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

std::vector<int> Ped::Tagent_collection::positionsX;
std::vector<int> Ped::Tagent_collection::positionsY;

Ped::Tagent_collection::Tagent_collection(int posX, int posY) {
	Ped::Tagent_collection::init(posX, posY);
}

Ped::Tagent_collection::Tagent_collection(double posX, double posY) {
	Ped::Tagent_collection::init((int)round(posX), (int)round(posY));
}

void Ped::Tagent_collection::init(int posX, int posY) {
	x = posX;
	y = posY;

	destination = NULL;
	lastDestination = NULL;
}

// TODO 
// Setters and getters
// -----------------------------------
void Ped::Tagent_collection::setX(int newX){}
void Ped::Tagent_collection::setY(int newY){}

std::vector<int> Ped::Tagent_collection::getDesiredX(){}
std::vector<int> Ped::Tagent_collection::getDesiredY(){}
//------------------------------------



void Ped::Tagent_collection::computeNextDesiredPositionScalar(std::vector<int>::iterator start,
															  std::vector<int>::iterator end) {
	destination = getNextDestination();
	if (destination == NULL) {
		// no destination, no need to
		// compute where to move to
		return;
	}

	double diffX = destination->getx() - getX();
	double diffY = destination->gety() - getY();
	double len = sqrt(diffX * diffX + diffY * diffY);
	desiredPositionX = (int)round(getX() + diffX / len);
	desiredPositionY = (int)round(getY() + diffY / len);
}

void Ped::Tagent_collection::addWaypoint(Twaypoint* wp) {
	waypoints.push_back(wp);
}

Ped::Twaypoint* Ped::Tagent_collection::getNextDestination() {
	Ped::Twaypoint* nextDestination = NULL;
	bool agentReachedDestination = false;

	if (destination != NULL) {
		// compute if agent reached its current destination
		double diffX = destination->getx() - getX();
		double diffY = destination->gety() - getY();
		double length = sqrt(diffX * diffX + diffY * diffY);
		agentReachedDestination = length < destination->getr();
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
