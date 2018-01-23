#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane,  string state, float speed) {
    this->lane_ = lane;
    this->state = state;
    this->max_speed_ = speed;
}


Vehicle::~Vehicle() {}


void Vehicle::choose_next_state(vector<vector<double>> sensor_fusion) {
    float cost;
    vector<float> costs;
    vector<string> possible_next_states = successor_states();
    if(debug) {std::cout << "state = "  << state << std::endl;}

    for(int i=0; i < possible_next_states.size(); i++)
    {
        string next_state = possible_next_states[i];
        cost = calculate_cost(*this, next_state, sensor_fusion);
        costs.push_back(cost);
        if(debug) {std::cout << "  next_state = "  << next_state << " - " << cost << std::endl;}

    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    state = possible_next_states[best_idx];
}

vector<string> Vehicle::successor_states() {
    vector<string> states;
    string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("KL");
        states.push_back("PLCL");
        states.push_back("PLCR");
        states.push_back("KLSU");
        states.push_back("KLSD");
    } else if (state.compare("PLCL") == 0) {
            states.push_back("LCL");
    } else if (state.compare("PLCR") == 0) {
            states.push_back("LCR");
    } else if (state.compare("LCL") == 0) {
            states.push_back("KL");
            states.push_back("LCL");
    } else if (state.compare("LCR") == 0) {
            states.push_back("KL");
            states.push_back("LCR");            
    }
    else if (state.compare("KLSU") == 0) {
            states.push_back("KL");
            states.push_back("KLSU");            
    }
    else if (state.compare("KLSD") == 0) {
            states.push_back("KL");
            states.push_back("KLSD");            
    }
    return states;
}

// Update vehicle state variables after new measurement data from the simulator arrives
  void Vehicle::update(double x, double y, double s, double d, double yaw, double speed, int prev_size)
  {
    x_ = x;
    y_ = y;
    s_ = s;
    d_ = d;
    yaw_ = yaw;
    speed_ = speed;
    prev_path_size_ = prev_size;
  }