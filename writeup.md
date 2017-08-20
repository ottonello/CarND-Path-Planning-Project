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


To generate smooth transitions between points, hence minimizing the `jerk` measuremente, we will generate a cubic spline from three calculated points in the road, spaced 30 meters. The cubic spline C++ implementation at http://kluge.in-chemnitz.de/opensource/spline/ will be used. First we generate the desired points and set them into
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

Here the `s` coordinate is incremented in 30 meters on each step, while the `d` position is generated according to the
desired lane location. Each lane is 4 meters wide, so to locate the car in the center of the lane we add 2 extra meters
to the coordinate. `getXY` returns the transformed `x`, `y` map coordinates, which we will transform later to car-relative
coordinates to simplify the calculations.

It's easy to create a spline using the points generated so far:
```
tk::spline s;
s.set_points(pts_x, pts_y);
```

Once we have a spline, we generate the points by spacing them so that the number of points
obtained above fit in the segment. It's easy to get the `y` coordinate of a point in the 
spline by just evaluating it on a known `x` coordinate:
```
double y_point = s(x_point);
```

Since the processing time is not infinitely short, the simulator continues to use points from
the last provided arrays as long as it doesn't receive new ones. To have a smooth running
car we reuse the remaining points from the last processing step, which are provided as input to the program, and only fill in the remaining ones so that we always return 50 coordinates.

Before adding the points to the vector we'll be returning to the simulator, we transform the points back to map-relative:
 ```
// transform back to map-relative coordinates
x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

x_point += ref_x;
y_point += ref_y;

next_x_vals.push_back(x_point);
next_y_vals.push_back(y_point);
```

Lane navigation
-------------
The implementation contains a very simple state machine in which the new state never depends 
on the current one, since it was determined more sophisticated steps were not required to achieve a passing implementation. 

We iterate over the sensor fusion data to determine the current location and speeds of cars around us:
```
for(int i = 0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = sensor_fusion[i][5];

    // Project next timeframe
    check_car_s += ((double) prev_size * time_delta * check_speed);
    double s_distance = check_car_s - car_s;

    if(is_on_lane(d, lane)) { // Same lane
        if(s_distance > 0 && s_distance < detection_distance) {
            car_in_front = true;
            // Car speed is in m/s
            car_in_front_speed = check_speed *2.24;
        }
    } else if (is_on_lane(d, lane-1)){ // Left lane
        if(s_distance > -detection_distance_back && s_distance < detection_distance) {
            car_on_left_lane = true;
        }
    } else if (is_on_lane(d, lane+1)) { // Right lane
        if(s_distance > -detection_distance_back && s_distance < detection_distance) {
            car_on_right_lane = true;
        }
    }
}
```

In this block we also project the next location of the car if it would continue moving over the same lane, which is a reasonable expectation. We use this position to determine in advance if the other vehicle will be in front of our car, and if it is close to us we set a flag. We also use a variable to store the speed of the car in front of us.

We also set a flag whenever we determine a car is on each of any of the adjacent lanes, and close in the `s` coordinate, both ahead and looking back.

Then a simple code block will determine the next state we want to be in from the state of the flags we set above:
```
if(car_in_front ) {
        if(lane > 0 && !car_on_left_lane) {
            state = CHANGE_LEFT;
        } else if (lane < 2 && !car_on_right_lane){
            state = CHANGE_RIGHT;
        } else if(ref_vel > car_in_front_speed){
            state = SLOW_DOWN;
        } else {
            state = KEEP_SPEED;
        }
} else {
    state = KEEP_LANE;
}
```

The logic is very simple here, if there are no cars in front we just keep the lane. Else, we'll consider changing lanes if there are no cars in any of the adjacent lanes in the observed region. We attempt passing on the left first, then we check the right. In case both lanes are taken, we just check whether the car in front is moving slower, and in that case we decide we should slow down.  

An action taking block follows, where we check the current state and do something based on it:

```
switch(state){
    case CHANGE_LEFT: lane--; break;
    case CHANGE_RIGHT: lane++; break;
    case KEEP_LANE:
        if(ref_vel < max_vel){
            ref_vel += 7 / .224*time_delta;
        }
        break;
    case SLOW_DOWN:
        ref_vel -= 4 / .224*time_delta;;
        break;
}
```

The speed delta was determined so that it doesn't cause us to exceed the maximum acceleration and jerk. We can calculate the maximum delta so that we don't exceed 10m/s^2 in one direction but mixed maneuvers can be tricky so we are better off safe here.

Possible improvement
-------

This implementation will usually drive incident-free almost indefinitely, but we're being on the safe side on some instances. For example, when considering changing lanes, we look back 10 meters on the adjacent lane in case any car is coming fast on that lane. In general we'll be able to out-accelerate those cars given this safe margin. We could be smarter and check the incoming speed or speed differential, and use a weighted system to determine the action to take on the next step. A more complex state machine could also help. We could have additional states for matching the speed of the cars when changing lanes.
