Path Planning Project Writeup
------

The objective of this project is to navigate a car through traffic on a highway. For this, the simulator
provides us with:
- The car's localization data:
	- `x` an `y` coordinates relative to the map
	- `s` an `d` Frenet coordinates relative to the map
	- Yaw
	- Speed
- Sensor fusion data, with the localization of the rest of the vehicles.

The output of the program should be a list of `x,y` points, which should indicate the next desired positions
of the car, relative to the map. Each 20 milliseconds the car is moved perfectly to the next indicated coordinates, 
so no controllers are required for this project, but we have constraints on the maximum normal, tangential and total
accelerations, and the `jerk` parameter which is measured as the rate of change of the total acceleration.
The remaining constraint(ignoring the position of other cars) is the maximum speed (50 mph). 

Movement
--------

To move the car, we must provide a new set of points which follows the path we desire the car to take.
This can be reduced to two problems:
- Provide a set of points which follow the lane line the car is on right now.
- Provide a set of points which allow the car to transition to the required lane line when it needs to switch.

The basic movement(straight line keeping the lane) is given by incrementing the position of the car in the `s` value in
Frenet coordinates, while keeping the `d` value the same. We woll generate a segment of 50 coordinate pairs with
the next locations. The points in the segment should be spaced in a way so that the desired speed is met. 
The number of points will be given by:
 
```
N = segment_length / (time_delta * velocity /2.24);
```

Here, `segment_length` is an arbitrary reference distance, `time_delta` is the time in which the car
will move between two points(given as 20ms.), and `velocity` is the target speed.


To generate smooth transitions between points, hence minimizing the `jerk`, we will generate a cubic spline from
three calculated points in the road, spaced 30 meters. First we generate the desired points and set them into
two vectors, for the `x` and `y` coordinates:

```
            // Generate spline points
            vector<int> steps = {30,60,90};
            for(int i = 0; i<steps.size(); i++){
                vector<double> next_wp = getXY(car_s + steps[i],(2+4*lane), map_waypoints_s,map_waypoints_x,map_waypoints_y);
                pts_x.push_back(next_wp[0]);
                pts_y.push_back(next_wp[1]);
            }
```