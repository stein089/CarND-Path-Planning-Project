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
  int lane_;
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

  bool debug = false;

  Vehicle();
  Vehicle(int lane, string state, float speed);

  virtual ~Vehicle();

  void choose_next_state(vector<vector<double>> sensor_fusion);

  vector<string> successor_states();

  void update(double x, double y, double s, double d, double yaw, double speed, int prev_size);
};

#endif