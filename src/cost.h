#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle & vehicle, string next_state, vector<vector<double>> sensor_fusion);

#endif