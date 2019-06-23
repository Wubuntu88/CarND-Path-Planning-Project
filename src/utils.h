/*
 * utils.h
 *
 *  Created on: Jun 2, 2019
 *      Author: williamgillespie
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <math.h>
#include <vector>
using std::vector;

double delta_t = 0.02;

int getLane(double car_d) {
  int lane = 2; // 0, 1, or 2
  // use car_d to determine current lane
  if (car_d < 4.0) {
    lane = 0;
  } else if (car_d < 8.0) {
    lane = 1;
  }
  return lane;
}

double getCarVelocity(vector<double> car) {
  double vx = car[3];
  double vy = car[4];
  return sqrt(vx*vx + vy*vy);
}

double getCheckCarS(vector<double> car, int previous_size) {
  double s = car[5];
//  double vx = car[3];
//  double vy = car[4];
//  double check_speed = sqrt(vx*vx + vy*vy);
  double check_speed = getCarVelocity(car);
  double check_car_s = s + ((double)previous_size*delta_t*check_speed);
  return check_car_s;
}

vector<double> getCarInFront(int lane_of_car, double car_s, int prev_size, int ahead_check_distance,
    vector<vector<double>> sensor_fusion) {
  vector<double> car_in_front;
  if(sensor_fusion.size() > 0) {
    vector<double> current_car_in_front;
    for(int i = 0; i < sensor_fusion.size(); ++i) {
      //if the car is in our lane
      vector<double> other_car = sensor_fusion[i];
      float other_car_d = other_car[6];
      double other_car_s = other_car[5];
      double check_car_s = getCheckCarS(other_car, prev_size);

      int other_car_lane = getLane(other_car_d);
      bool is_in_same_lane = lane_of_car == other_car_lane;
      bool is_ahead = check_car_s > car_s && (check_car_s-car_s < ahead_check_distance);
      if(is_in_same_lane && is_ahead) {
        current_car_in_front = other_car;
        car_in_front = current_car_in_front;
      }
    }
  }
  return car_in_front;
}
#endif /* UTILS_H_ */
