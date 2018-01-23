#include "cost.h"
#include "vehicle.h"
#include <string>
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

float calculate_cost(const Vehicle & vehicle, string next_state, vector<vector<double>> sensor_fusion) {
    float cost = 0.0;

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

               //   std::cout << "KL check_car_s: " <<   check_car_s << " vehicle.s: " << vehicle.s_ << std::endl;

                if( check_car_s > vehicle.s_ && check_car_s < (vehicle.s_+50))
                {
                    cost = cost + 500;
                }
            }
        }
        
        if(vehicle.state.compare("LCR") == 0)
        {
            // Back in lane again -> exit state then
            if(vehicle.d_ < (2+4*vehicle.lane_+1) && vehicle.d_  > (2+4*vehicle.lane_-1))
            {
                cost = 0;
            }
        }
        else if(vehicle.state.compare("LCL") == 0)
        {
            // Back in lane again - switch back to "keep lane"
            if(vehicle.d_ < (2+4*vehicle.lane_+1) && vehicle.d_  > (2+4*vehicle.lane_-1))
            {
                cost = 0;
            }
        }
        else if(vehicle.state.compare("KLSD") == 0 || vehicle.state.compare("KLSU") == 0)
        {
            // Target speed reached - switch back to "keep lane"
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


                   // std::cout << "PLCL check_car_s: " <<   check_car_s << " my_car_s: " << my_car_s 
                  //        << "check_speed: " << check_speed*2.24 << " vehicle.speed:" << vehicle.speed_ << std::endl;

                    if( check_car_s > (my_car_s-20) && check_car_s < (my_car_s+20))
                    {
                      //  std::cout << " PLCL - if 1" << std::endl;
                        cost = cost + 100;
                    }

                    if( check_speed*2.24 > vehicle.speed_ && check_car_s > 
                        (my_car_s-35) && check_car_s < (my_car_s-20) )
                    {
                       // std::cout << " PLCL - if 2" << std::endl;
                        cost = cost + 50;
                    }

                    if( check_speed*2.24 < vehicle.speed_ && check_car_s > 
                        (my_car_s+20) && check_car_s < (my_car_s+35) )
                    {
                       // std::cout << " PLCL - if 3" << std::endl;
                        cost = cost + 50;
                    }

                    if( check_car_s > (my_car_s+35) )
                    {
                       // std::cout << " PLCL - if 4" << std::endl;
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

                  //std::cout << "PLCR check_car_s: " <<   check_car_s << " my_car_s: " << my_car_s 
                   //       << "check_speed: " << check_speed*2.24 << " vehicle.speed:" << vehicle.speed_ << std::endl;

                    if( check_car_s > (my_car_s-20) && check_car_s < (my_car_s+20))
                    {
                        cost = cost + 100;
                    }

                    if( check_speed*2.24 > vehicle.speed_ && check_car_s > 
                        (my_car_s-35) && check_car_s < (my_car_s-20) )
                    {
                        cost = cost + 50;
                    }

                    if( check_speed*2.24 < vehicle.speed_ && check_car_s > 
                        (my_car_s+20) && check_car_s < (my_car_s+35) )
                    {
                        cost = cost + 50;
                    }

                    if( check_car_s > (my_car_s+35) )
                    {
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
