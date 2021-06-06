[//]: # (Image References)

[image1]: ./StateChart.jpg "Statechart of behavior planing module"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Summary
This project deals with the problems of behavior planning and trajectory planing for highway driving scenario.

Behavior planing includes the interpretation of the modeled world so rounding the ego vehicle and the planing of appropriate actions based on the ego vehicles goals and the environmental conditions.

Trajectory or Path Planing describes the generation of executable commands to a vehicles longitudinal and lateral control. In the case of this project, this is a trajectory which describes positions over time, neglecting the vehicle dynamics.

#### Behavior planning
The underlying state chart of the behavior planing module consists of the following states. 
![alt text][image1]


I am explicitly distinguishing between approaching and following a leading vehicle. In the 'approaching' state, which will be triggered when reaching the constant approach distance, the ego vehicles speed is linearly reduced until both vehicles match speed. Once that state is done and the following distance is reached, or suddenly another car changes to our lane below the following distance, speed will be further increased to increase the distance. In both the 'approaching' and following state, the ego vehicle will try to change lanes and take over slower vehicles.

Behavior planing can be found in lines 135 - 267 in main.cpp. In the first step the model of the surrounding world is check which would be supplied to the behavior planer by the sensor fusion module. Information gathered here will later on be used for decision-making. I am first checking for cars in the same lane as the ego vehicle and the distance is below the 'approach' or 'follow' distance. If that is the case, I am checking if a lane change to the left or right is allowed. The criterion for allowing a lane change is met if the difference in the longitudinal distance after one sample step is above the minimum allowed distance between the ego vehicle and vehicles in the lane directly left or right of the current lane. The prediction of the position of ego vehicle and potential vehicles on the neighboring lanes is done under the assumption that all vehicles will keep their speed. If now lane change is allowed we would transition to the 'approaching' or 'following' state dependent on the longitudinal distance to the leading vehicle.

With the information based on the sensor fusion, we will now issue commands to the path planer (lines 218 - 267). If we encountered now vehicle below the 'approaching' distance, the desired lane will be kept and the ego vehicles velocity set point will be increased until the speed limit of 50 mph is reached. If below 'following' or 'approaching' distance and a lane change is allowed, the current speed will be kept, and the desired lane will be changed. Lane changes to the left lane are implicitly prioritized. A lane change is only allowed if no lane change is currently ongoing. This forces the car to finish lane changes and prevents frequent back and forth between lanes in case there is a car in the current and desired lane with roughly the same distance. A lane change is regarded as finished if the ego vehicle is within a 10% corridor from the lanes center. If we are within the 'approaching' distance, the velocity set point will be decreased until ego vehicle and leading vehicle have zero relative velocity. If the 'following' distance is reached, the ego vehicles speed will be decreased further to increase distance to a safe level.

#### Trajectory planning
The trajectory planer is a modified version of the approach presented in the FAQ video. Relevant code lines are 266 - 377 in main.cpp. 

If  points are queued up which have not been realized in the last sample step, those will be queued up again. Contrary to FAQ, I chose to only queue up a maximum up to 10 points. This helps to be able to react faster to leading vehicles and reduced the collisions significantly, e.g. if a car suddenly changes to the ego vehicles current lane. 

For updating the trajectory, the spline library is used. First a spline is fitted through five points. Points one and two are either the last two points which have not yet been realized or in case no points are queued up the current and previous position of the ego vehicle. This ensures that the trajectory is continuous and updates to the trajectory will be tangential to the path in the past. The other three points are in fixed distances along the maps reference line. I chose a spacing for the s-coordinates of 30, 60, and 90 meters. Dependent on the desired lane, the d-coordinates of those points will be determined to either be in the same or desired lane. The splines are functions y(x) where x and y are coordinates relative to the last point from the previous sample step or the current position. Pose is determined by the current or future yaw angle.

The spline will now be sampled to provide position updates, the simulator will realize each provided point every 20 ms. With the sampling method presented in the FAQ I encountered some issues which are caused by the linear approximation which is used to calculate x values. In case of splines with increasing curvature this method will get increasingly inaccurate which prevents points to be spaced evenly along the spline. Consequentially this caused vehicle speeds higher than the allowed 50 mph on high-curvature segments of the track or acceleration and jerk outside the allowed margin when changing lanes. 

I am instead using a simple method based on bisection. The algorithm can be found in helpers.h line 202 in the 'sampleTrajectory' function. I am basically calculating where the spline intersects a circle, with a radius which is determined by the distance the ego vehicle should travel in one simulator step. Or in other words, by the product of desired velocity and the simulators sample time (20ms). The algorithm is angle based and would start to calculate the splines y value at 45 deg of the circle, or to be precise at x = cos(45 deg). The boundaries of the angle are initialized between 0 and 90 deg. After that, the euclidean distance between the new and preceding x/y coordinates will be calculated. If that distance is smaller than the desired radius the point of intersection is at a smaller angle, the upper boundary will be set to the current angle and the angle will set to the middle of the new interval. In the same way, if the distance is bigger the intersection is at a bigger angle, the lower boundary will be updated with the current angle and angle will be set to the middle of the new interval as well. 

### Reflection
The algorithm is able to drive the ego vehicle for at least the required distance while keeping the desired speed, not causing a collision or violating the acceleration, speed and jerk limits, at least for most of my simulation runs. However, there is certainly room for improvement.

The implementation of decision-making is more or less unstructured, which will make it hard to maintain and extend the code. For better maintainability it will be best to use a recognizable design pattern for a state machine where it is more obvious how transitions are triggered and what actions are carried out in each state. 

Currently, the prediction of the ego vehicle and non-ego vehicles are carried out with the assumption that they will keep constant speed. As this is not the case in the real world, better predictions should be possible based on a motion model, e.g. by using a Kalman filter.

Also, the spline based approach for the calculation of the trajectory doesn't allow a direct consideration of jerk or acceleration boundary conditions. As an alternative to the spline based approach, path planing can be solved, e.g. with the jerk minimizing polynomial method presented in the lectures.




