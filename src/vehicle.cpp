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
    lane_direction["PLCL"] = 1;
    lane_direction["LCL"] = 1;    
    lane_direction["LCR"] = -1;
    lane_direction["PLCR"] = -1;
}


Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle> > predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

 
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle> > final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];   */
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

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle> > predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
      //  trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
      //  trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
       // trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        //trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}


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