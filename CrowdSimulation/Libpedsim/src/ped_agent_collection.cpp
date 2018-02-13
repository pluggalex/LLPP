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

void Ped::Tagent_collection::desiredUpdateVector(int start, int end, 
													std::vector<float>& diffX,
													std::vector<float>& diffY,
													std::vector<float>& len){
	__m128 A, B, C, D;
	for (int index = start; index <= end - 4; index += 4){
		//Round and save to desired
		A = _mm_load_ps(&diffX[index - start]);
		B = _mm_load_ps(&len[index - start]);
		C = _mm_div_ps(A, B);

		A = _mm_load_ps(&x[index]);
		B = _mm_add_ps(A, C);
		D = _mm_round_ps(B, _MM_FROUND_TO_NEAREST_INT);

		_mm_store_ps(&desiredPositionX[index], D);

		//Round and save to desired
		A = _mm_load_ps(&diffY[index - start]);
		B = _mm_load_ps(&len[index - start]);
		C = _mm_div_ps(A, B);

		A = _mm_load_ps(&y[index]);
		B = _mm_add_ps(A, C);
		D = _mm_round_ps(B, _MM_FROUND_TO_NEAREST_INT);

		_mm_store_ps(&desiredPositionY[index], D);
	}
}


void Ped::Tagent_collection::computeDiff(int start, int end,
										std::vector<float>& diffX,
										std::vector<float>& diffX_pwr2,
										std::vector<float>& diffY,
										std::vector<float>& diffY_pwr2,
										std::vector<float>& len){

	__m128 A, B, C, D;

	for (int index = start; index <= end - 4; index += 4){
		//Calculate diffX
		A = _mm_load_ps(&destinations->getDestRefX()[index]);
		B = _mm_load_ps(&x[index]);
		C = _mm_sub_ps(A, B);
		D = _mm_mul_ps(C, C);
		_mm_store_ps(&diffX[index - start], C);
		_mm_store_ps(&diffX_pwr2[index - start], D);

		//Calculate diffY
		A = _mm_load_ps(&destinations->getDestRefY()[index]);
		B = _mm_load_ps(&y[index]);
		C = _mm_sub_ps(A, B);
		D = _mm_mul_ps(C, C);
		_mm_store_ps(&diffY[index - start], C);
		_mm_store_ps(&diffY_pwr2[index - start], D);

		//Len from sqrt
		A = _mm_load_ps(&diffX_pwr2[index - start]);
		B = _mm_load_ps(&diffY_pwr2[index - start]);
		C = _mm_add_ps(A, B);
		D = _mm_sqrt_ps(C);
		_mm_store_ps(&len[index - start], D);
	}
}
void Ped::Tagent_collection::computeNextDesiredPositionVector(int start, int end) {
	std::vector<float> diffX(end-start, 0);
	std::vector<float> diffX_pwr2(end-start, 0);
	std::vector<float> diffY(end-start, 0);
	std::vector<float> diffY_pwr2(end-start, 0);
	std::vector<float> len(end-start, 0);	

	computeDiff(start, end, diffX, diffX_pwr2, diffY, diffY_pwr2, len);

	for (int index = start; index < end; index++){
		if (len[index - start] < destinations->getDestRefR()[index]){
			destinations->rotateQueue(index);
		}
	}

	computeDiff(start, end, diffX, diffX_pwr2, diffY, diffY_pwr2, len);
	desiredUpdateVector(start, end, diffX, diffY, len);
	/*for (int index = start; index < end; index++){
		desiredPositionX[index] = (int)round(x[index] + diffX[index - start] / len[index - start]);
		desiredPositionY[index] = (int)round(y[index] + diffY[index - start] / len[index - start]);
	}*/
}

void Ped::Tagent_collection::addWaypoint(Twaypoint* wp) {
	destinations->addWaypoint(wp, 0, x.size());
}


Ped::Tagent_collection& Ped::Tagent_collection::operator += (const Ped::Tagent_collection& rhs){
	std::vector<float> rhsX = rhs.getX();
	std::vector<float> rhsY = rhs.getY();
	x.insert(x.end(), rhsX.begin(), rhsX.end());
	y.insert(y.end(), rhsY.begin(), rhsY.end());
	
	std::vector<float> desiredX = rhs.getDesiredX();
	std::vector<float> desiredY = rhs.getDesiredY();
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
