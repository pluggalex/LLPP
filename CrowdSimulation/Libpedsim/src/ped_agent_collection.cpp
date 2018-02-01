//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//

#include <math.h>
#include <algorithm>
#include "ped_agent_collection.h"

// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

Ped::Tagent_collection::Tagent_collection() : x(0), y(0) {destinations = std::make_unique<Ped::waypoints>(); }

void Ped::Tagent_collection::addAgent(int posX, int posY){
	x.push_back(posX);
	y.push_back(posY);
	desiredPositionX.push_back(posX);
	desiredPositionY.push_back(posY);

	destinations->setAgents(x.size());
}


// TODO 
// Setters and getters
// -----------------------------------
//void Ped::Tagent_collection::setX(int newX){}
//void Ped::Tagent_collection::setY(int newY){}

//std::vector<int> Ped::Tagent_collection::getDesiredX(){}
//std::vector<int> Ped::Tagent_collection::getDesiredY(){}
//------------------------------------


void Ped::Tagent_collection::updateFromDesired(int start, int end){
	std::copy(desiredPositionX.begin() + start, desiredPositionX.begin()+end, x.begin()+start);
	std::copy(desiredPositionY.begin() + start, desiredPositionY.begin() + end, y.begin() + start);
}


void Ped::Tagent_collection::computeNextDesiredPositionScalar(int start, int end) {

	//setNextDestinationScalar(start, end);

	for (int index = start; index < end; index++){
		double diffX = destinations->getDestRefX()[index] - x[index];
		double diffY = destinations->getDestRefY()[index] - y[index];
		double len = sqrt(diffX * diffX + diffY * diffY);

		if (len < destinations->getDestRefR()[index]){
			destinations->rotateQueue(index);
		}
		else{
			desiredPositionX[index] = (int)round(x[index] + diffX / len);
			desiredPositionY[index] = (int)round(y[index] + diffY / len);
		}
	}
}

void Ped::Tagent_collection::addWaypoint(Twaypoint* wp) {
	destinations->addWaypoint(wp, 0, x.size());
}


Ped::Tagent_collection& Ped::Tagent_collection::operator += (const Ped::Tagent_collection& rhs){
	std::vector<int> rhsX = rhs.getX();
	std::vector<int> rhsY = rhs.getY();
	x.insert(x.end(), rhsX.begin(), rhsX.end());
	y.insert(y.end(), rhsY.begin(), rhsY.end());
	
	std::vector<int> desiredX = rhs.getDesiredX();
	std::vector<int> desiredY = rhs.getDesiredY();
	desiredPositionX.insert(desiredPositionX.end(), desiredX.begin(), desiredX.end());
	desiredPositionY.insert(desiredPositionY.end(), desiredY.begin(), desiredY.end());

	Ped::waypoints* rhsDest = rhs.borrowDestinations();
	*destinations += *rhsDest;

	return *this;
}

/*
* Potentially not needed, maybe :)
*/
void Ped::Tagent_collection::setNextDestinationScalar(int start, int end) {
	int agentReachedDestination;
	for (int i = start; i < end; i++){
		// compute if agent reached its current destination
		double diffX = destinations->getDestRefX()[i] - x[i];
		double diffY = destinations->getDestRefY()[i] - y[i];
		double length = sqrt(diffX * diffX + diffY * diffY);
		agentReachedDestination = length < destinations->getDestRefR()[i];

		if (agentReachedDestination)
			destinations->rotateQueue(i);
	}
}
