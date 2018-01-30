//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_coordinate_collection.h"

#include <cmath>

// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

// initialize static variables
//int Ped::coordinate_collection::staticid = 0;

// Constructor: Sets some intial values. The agent has to pass within the given radius.
Ped::coordinate_collection::coordinate_collection(double px, double py, double pr) {x.push_back(px); y.push_back(py); r.push_back(pr); };

// Constructor - sets the most basic parameters.
Ped::coordinate_collection::coordinate_collection() : x(0), y(0), r(1) {};

Ped::coordinate_collection::~coordinate_collection() {};


