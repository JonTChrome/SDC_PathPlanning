#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"

using namespace std;


#define JERK_COST_WEIGHT 99999

// #define BUFFER_COST_WEIGHT 10
// #define IN_LANE_BUFFER_COST_WEIGHT 1000
// #define EFFICIENCY_COST_WEIGHT 10000
// #define NOT_MIDDLE_LANE_COST_WEIGHT 100

double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
	double total_cost = 0;


	return total_cost;
}