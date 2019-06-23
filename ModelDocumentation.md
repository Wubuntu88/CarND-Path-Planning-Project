# CarND-Path-Planning-Project Model Documentation
This document explains the model of my project.
It is composed of several parts
1) Car Following Logic
2) Lane Changing Logic
3) Path Planner using the Spline method from the tutorial

###1) Car following logic
For following a car, there are several things the models considers.
The model checks to see if the car is too close to a lead car in front.
If this is the case a flag is set, and the lead car velocity is recorded.
The model also checks to see if the lead car is at an 'emergency distance'.

The model will choose to change lane if the lead car is too close.
If the car cannot change lane when it is too close, 
it will proceed with the car following logic.

Based on this information, The model makes several decisions:

1: If the car is at an emergency distance to the car in front, 
the car will decelerate more quickly.
```c++
if(emergency_close) {
    std::cout << "emergency deceleration." << std::endl;
    ref_vel -= .33;
}
```
2: If the controlled car and the lead car have an absolute difference of velocities that is greater than .224 m/s,
then we will decrease the velocity for 0.224 m/s.
```c++
else if(abs(lead_car_velocity - ref_vel) > .224) {
    std::cout << "decelerating, car in lane: " << lane << std::endl;
    ref_vel -= .224;
}
```
3: If the absolute difference between the controlled car and lead car is less than .224 m/s,
then the velocity of the car is set equal to that of the lead car.
The reason we have this check is to make for a smooth change of the velocity.
If the velocity is set to that of the lead car when the velocities are far apart,
The car will accelerate too fast and cause excessive jerk.
```c++
else {
    std::cout << "following, in lane: " << lane << std::endl;
    ref_vel = lead_car_velocity;
}
```

###2) Lane Changing Logic
The controlled car will decide to change lanes when it is too close to a lead car,
or when it wants to return to the middle lane and is free to do so.
First, the case of the car being too close to a lead car will be discussed.

All of the other cars are iterated over, and for each one of the other cars,
the car is checked if it is in the other lane, 
and whether the predicted s value is within a certain distance ahead and behind
of the controlled car.  If the other car is in the other lane and is too close,
a 'can_turn_left' or 'can_turn_right' flag is set to false.
These flags will be used to decide which lane to turn into.
If the car is in the center and both lanes are available,
the car will turn into the left lane by default.
```c++
for(int i = 0; i < sensor_fusion.size(); ++i) {
    //if the car is in our lane
    vector<double> other_car = sensor_fusion[i];
    double other_car_s = other_car[5];
    double other_car_d = other_car[6];
    
    // calculates s value in the future
    
    double other_car_future_s = getCheckCarS(other_car, prev_size);
    
    int other_car_lane = getLane(other_car_d);
    
    bool other_car_is_too_close = other_car_future_s < car_s + safe_distance_ahead && other_car_future_s > car_s - safe_distance_behind;
    
    bool other_car_is_in_left_lane = lane - other_car_lane == 1;
    bool other_car_is_in_right_lane = other_car_lane - lane == 1;
    if (other_car_is_in_left_lane && other_car_is_too_close) {
      can_turn_left = false;
    }
    if (other_car_is_in_right_lane && other_car_is_too_close) {
      can_turn_right = false;
    }
}

if(too_close) {
    if (lane > 0 && can_turn_left) {
      lane -= 1;
      std::cout << "lane change left into lane: " << lane << std::endl;
      if(emergency_close) {
        std::cout << "emergency deceleration." << std::endl;
        ref_vel -= .33;
      }
    } else if (lane < 2 && can_turn_right){
      lane += 1;
      std::cout << "lane change right into lane: " << lane << std::endl;
      if(emergency_close) {
        std::cout << "emergency deceleration." << std::endl;
        ref_vel -= .33;
      }
}
.
.
.
```
###3) Path planning using the spline method from the tutorial.
The spline path planner from the tutorial generates a spline using 2 points 
that are based on where the car is and the last position of the car,
as well as 3 points out ahead of the car.
The 3 points out ahead of the car are generated using the map's way points (that already exist),
and the lane the car will be staying in or turning into.  Once these points are determined,
a spline is created, and some 50 points are generated such that the car travels at the 
desired reference velocity.
