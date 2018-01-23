#include "cost.h"
#include "vehicle.h"
#include <string>
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);


float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.

    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }    */
    return 0;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
   

    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = vehicle.target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }
    
    float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;
 */
    return 0;
}

float lane_speed(const map<int, vector<Vehicle> > & predictions, int lane) {
    /*
    All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.

    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.lane_ == lane && key != -1) {
            return vehicle.v;
        }
    }    */
    //Found no vehicle in the lane
    return -1.0;
}

float calculate_cost(const Vehicle & vehicle, string next_state, vector<vector<double>> sensor_fusion) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    //map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    //Add additional cost functions here.
   // vector< function<float(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = {goal_distance_cost, inefficiency_cost};
   // vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
   // for (int i = 0; i < cf_list.size(); i++) {
   //     float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
   //     cost += new_cost;
  //  }

	if(next_state.compare("KL") == 0)
    {
        cost = 10;
        for(int i=0; i < sensor_fusion.size(); i++)
        {
            float d = sensor_fusion[i][6];
            
            if(d < (2+4*vehicle.lane_+2) && d > (2+4*vehicle.lane_-2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s= sensor_fusion[i][5];
                check_car_s += ((double)vehicle.prev_path_size_*0.02*check_speed);

                  std::cout << "KL check_car_s: " <<   check_car_s << " vehicle.s: " << vehicle.s_ << std::endl;



                if( check_car_s > vehicle.s_ && check_car_s < (vehicle.s_+50))
                {
                    // another car on the road
                    cost = cost + 500;
                }
            }
        }
        
        if(vehicle.state.compare("LCR") == 0)
        {
            // Back in line again -> exit state then
            if(vehicle.d_ < (2+4*vehicle.lane_+1) && vehicle.d_  > (2+4*vehicle.lane_-1))
            {
                cost = 0;
            }
        }
        else if(vehicle.state.compare("LCL") == 0)
        {
            // Back in line again -> exit state then
            if(vehicle.d_ < (2+4*vehicle.lane_+1) && vehicle.d_  > (2+4*vehicle.lane_-1))
            {
                cost = 0;
            }
        }
        else if(vehicle.state.compare("KLSD") == 0 || vehicle.state.compare("KLSU") == 0)
        {
            // Back in line again -> exit state then
            if( abs(vehicle.target_speed_ - vehicle.speed_) < 1)
            {
                cost = 0;
            }
        }
    }
    else if(next_state.compare("PLCL") == 0)
    {
        if (vehicle.lane_ > 0)
        {
            cost = cost + 20;
            for(int i=0; i < sensor_fusion.size(); i++)
            {
                float d = sensor_fusion[i][6];
                
                if(d < (2+4*(vehicle.lane_-1)+2) && d > (2+4*(vehicle.lane_-1)-2))
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx+vy*vy);
                    double check_car_s= sensor_fusion[i][5];
                    check_car_s += ((double)vehicle.prev_path_size_*0.02*check_speed);
                    double my_car_s = vehicle.s_+ vehicle.prev_path_size_*0.02*vehicle.speed_/2.24;


                    std::cout << "PLCL check_car_s: " <<   check_car_s << " my_car_s: " << my_car_s 
                          << "check_speed: " << check_speed*2.24 << " vehicle.speed:" << vehicle.speed_ << std::endl;

                    if( check_car_s > (my_car_s-20) && check_car_s < (my_car_s+20))
                    {
                        std::cout << " PLCL - if 1" << std::endl;
                        cost = cost + 100;
                    }

                    if( check_speed*2.24 > vehicle.speed_ && check_car_s > 
                        (my_car_s-35) && check_car_s < (my_car_s-20) )
                    {
                        std::cout << " PLCL - if 2" << std::endl;
                        cost = cost + 50;
                    }

                    if( check_speed*2.24 < vehicle.speed_ && check_car_s > 
                        (my_car_s+20) && check_car_s < (my_car_s+35) )
                    {
                        std::cout << " PLCL - if 3" << std::endl;
                        cost = cost + 50;
                    }

                    if( check_car_s > (my_car_s+35) )
                    {
                        std::cout << " PLCL - if 4" << std::endl;
                        cost = cost + 5*(exp(-check_car_s/(my_car_s+35)));
                    }

                }
            }
        }
        else
        {
            cost = 500;
        }
    }
    else if(next_state.compare("PLCR") == 0)
    {
        if (vehicle.lane_ < 2)
        {
            cost = cost + 20;
            for(int i=0; i < sensor_fusion.size(); i++)
            {
                float d = sensor_fusion[i][6];
                
                if(d < (2+4*(vehicle.lane_+1)+2) && d > (2+4*(vehicle.lane_+1)-2))
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx+vy*vy);
                    double check_car_s= sensor_fusion[i][5];
                    check_car_s += ((double)vehicle.prev_path_size_*0.02*check_speed);
                    double my_car_s = vehicle.s_+ vehicle.prev_path_size_*0.02*vehicle.speed_/2.24;

                  std::cout << "PLCR check_car_s: " <<   check_car_s << " my_car_s: " << my_car_s 
                          << "check_speed: " << check_speed*2.24 << " vehicle.speed:" << vehicle.speed_ << std::endl;

                    if( check_car_s > (my_car_s-20) && check_car_s < (my_car_s+20))
                    {
                        cost = cost + 100;
                        std::cout << " PLCR - if 2" << std::endl;
                    }

                    if( check_speed*2.24 > vehicle.speed_ && check_car_s > 
                        (my_car_s-35) && check_car_s < (my_car_s-20) )
                    {
                        std::cout << " PLCR - if 2" << std::endl;
                        cost = cost + 50;
                    }

                    if( check_speed*2.24 < vehicle.speed_ && check_car_s > 
                        (my_car_s+20) && check_car_s < (my_car_s+35) )
                    {
                        std::cout << " PLCR - if 3" << std::endl;
                        cost = cost + 50;
                    }

                    if( check_car_s > (my_car_s+35) )
                    {
                        std::cout << " PLCL - if 4" << std::endl;
                        cost = cost + 5*(exp(-check_car_s/(my_car_s+35)));
                    }
                
                }
            }
        }
        else
        {
            cost = 500;
        }
    }

	else if(next_state.compare("KLSD") == 0)
    {
        cost = cost + 50;
    }
	else if(next_state.compare("KLSU") == 0)
    {
        if( (vehicle.max_speed_ - vehicle.speed_) < 2)
        {
            cost = cost + 100;
        }

        for(int i=0; i < sensor_fusion.size(); i++)
        {
            float d = sensor_fusion[i][6];
            
            if(d < (2+4*vehicle.lane_+2) && d > (2+4*vehicle.lane_-2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s= sensor_fusion[i][5];
                check_car_s += ((double)vehicle.prev_path_size_*0.02*check_speed);
                if( check_car_s > vehicle.s_ && (check_car_s-vehicle.s_) < 50)
                {
                    // another car on the road
                    cost = cost + 600;
                }
            }
        }
    }


    return cost;
}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.
    distance_to_goal: the distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.

    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    float intended_lane;

    if (trajectory_last.state.compare("PLCL") == 0) {
        intended_lane = trajectory_last.lane_ + 1;
    } else if (trajectory_last.state.compare("PLCR") == 0) {
        intended_lane = trajectory_last.lane_ - 1;
    } else {
        intended_lane = trajectory_last.lane_;
    }

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane_;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;    */
}

