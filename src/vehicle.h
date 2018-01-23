#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction;

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane_;

  int s;

  float v;

  float a;



  int lanes_available;

  float max_acceleration;

  int goal_lane;

  int goal_s;


    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double speed_;
    int prev_path_size_;
    double target_speed_;
    double max_speed_;

  string state;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, string state, float speed);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle> > predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle> > predictions);

  void update(double x, double y, double s, double d, double yaw, double speed, int prev_size);




};

#endif