# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
Alistair Kirk November 2017

---
[//]: # (Image References)

[FSM]: FSM.png "Finite State Machine"

# Writeup and Reflection

## Project Introduction
This project explores the creation of Behavioural Path Planning for autonomous vehicles for use with the Term 3 Simulator. The goal is to automate the behaviour of the so called 'ego' vehicle around the simulated highway track loop, while navigating simulated random traffic spread over three lanes. The 'ego' car starts in the middle lane with zero velocity, and simulated traffic moving.

Sensor fusion data is fed into the program from the simulator that gives a list of all other detected vehicles on the road, including their global coordinates, and Frenet coordinates (s,d) that measure the distance s along the track, at a distance d from the median. Additional sensor data of the ego vehicle is provided including its global map coordinates, and Frenet coordinates.

This program parses the sensor fusion data and creates a list of vehicles using a Vehicle class. Each vehicle then generates predictive trajectories called 'predictions' which are then used in comparing ego vehicle predicted trajectories. This all results in collision avoidance and path planning for the ego vehicle by adjusting the acceleration of the vehicle for each timestep. Each timestep for the simulator is equivalent to 0.2 seconds.

Videos of the functioning Path Planning project can be found on Youtube.
*[self driving car nanodegree program Path Planning Project: Shows the car successfully driving around the test track.](https://youtu.be/aprbBvsAWiw)

## The Model

The majority of the Path Planning theory and code style was adapted from the Udacity course. A finite state machine (FSM) concept was used to model the behaviour states and transition logic for the path planning vehicle. The FSM used in this project is slightly different to that recommended in the Udacity course in that the ego vehicle may conduct a lane change without 'preparing' for a lane change. Drawbacks of this project's FSM are discussed throughout, future improvements would remove the ability to lane change without preparing for a lane change.

From the Path Planning module the general FSM layout is shown below:
![FSM][FSM]

The project uses the above FSM logic to create a set of paths for the ego vehicle, that are then stitched together as a series of coordinates using cubic spline interpolation (using available [spline.h](http://kluge.in-chemnitz.de/opensource/spline/) code). The knots of the spline are spaced according to the ego vehicle's speed to ensure smooth transitions between paths, and to minimize jerk and max acceleration.

A drawback in the Term 3 Simulator is that it appears to measure max jerk and acceleration over a single timestep (0.2 seconds), and there are very rare cases where the jerk limit will be exceeded, for example if the ego vehicle is attempting to make a lane change across two lanes at maximum speed while also turning on a sharp corner.

The ego vehicle transitions between the states by minimizing a set of calculated Cost Functions, that determine the most optimal state to be in. As shown in the figure, after being 'ready', the vehicle will try to keep its desired lane, and perform lane changes corresponding to the selected least cost function. The state and cost function calculation are discussed next.

### Concepts for Each State
#### Keep Lane

The ego vehicle will maintain the desired target speed (just under the preset legal speed limit of 50 mph), however if the ego vehicle is forced to slow down in its current lane, due to a slower vehicle ahead, the cost function for staying in a slow moving lane increases proportional to the difference in current speed and desired speed. If the cost increases above the Lane Change or Prepare Lane Change states, the state with the minimum cost is then selected.

#### Lane Change Left and Right (LCL / LCL)

In this state the ego vehicle determines the cost of committing a lane change, and increases the cost if there are vehicles nearby in that lane, which discourages lane changes that could result in collisions. This state logic works correctly for the majority of the driving in the simulator, and results in smooth transitions between lanes.

There are rare occasions in this code where the lane change algorithm initially detects collision avoidance in the adjacent lane, but sees that the second lane over is clear (for example going from lane 3 to lane 1, while another vehicle is in lane 2 travelling at the same speed as ego vehicle), and creates a path that either comes very close to a collision with the adjacent vehicle, or just grazes the vehicle. 

This error in logic has proven very difficult to debug because it is rare to occur in the simulator, and there is a significant amount of data needed to investigate and resolve the issue. Future improvements to the simulator would include an ability to set up test cases, allowing the user to dictate how many cars are on the road and what speeds they travel, whether they change lanes, etc. The current implementation of the code works for the rubric of this course, but will eventually fail if left to run infinitely.

#### Prepare for Lane Change Left and Right (PLCR / PLCL)

Currently when in this state, the code correctly identifies the nearest vehicle in the adjacent lane, and adjusts the ego vehicle speed to match that of the target. Admittedly the PLC state requires further development, but further time to spend on this project was very limited. The proposed future PLC logic would work as follows:

		1. Look at cars in goal lane.
		2. See if the gap is large enough, then pick lead car to merge behind (set goal_s).
		3. Adjust speed to match goal_s, or lead car minus a buffer.
		4. Check if Danger Close (pacing a car in current lane) 
			a. If new acceleration results in velocity > car ahead Then override acc to match car velocity ahead	

## The Code

Here the framework of the code is discussed:

### main.cpp

The ego vehicle begins by initializing a number of variables used to guide the program logic, including the simulator lanes on the highway,
 a desired goal lane (here selected as the middle lane [1] to increase available lane change options, and the course speed limits and max desired acceleration for the ego vehicle. All units are converted to SI for consistency.

```
// Init car configuration variables
  int numlanes = 3;
  double goal_s = -1;
  int goal_lane = 1;
  double MAX_ACCEL = 8.0; //m/s^2, just under 1G
  double SPEED_LIMIT = 49.5; // set in mph
  SPEED_LIMIT *= 0.44704; // convert to m/s
```

The ego vehicle is instantiated from the Vehicle class, and initialized with the configuration variables, including setting the ego vehicle velocity and acceleration to 0. This corresponds to the FSM 'Ready' state.

The sensor fusion data is parsed into C++ maps of predicted trajectories for each vehicle. The predictions are passed to the ego vehicle instance's `update_state`. The `update_state` function calculates the minimum cost state to be in and changes the desired state of the vehicle in the instance.

The ego vehicle instance then realizes the desired state using the `realize_state` function. This function updates the desired lane and acceleration of the ego vehicle. For example, if Keep Lane state, it will travel at maximum speed (SPEED_LIMIT) unless there is a car ahead, and it will slow down to avoid collision.

Once the lane and acceleration for the given timestep is realized, the ego vehicle velocity is updated based on standard kinematic equations:

```
ego.v = ego.v + ego.a*0.02;
```

Finally, the ego vehicle trajectory is generated as a set of x and y global points, stitching the last timestep endpoint for smoothing, and using cubic spline interpolation.


### vehicle.cpp / vehicle.h

A Vehicle Class is created as a template for not only the ego vehicle but also the other vehicles on the road. 

`update_state(...)` calculates trajectories for the current vehicle and set of allowable states. The trajectories of the ego vehicle states, and the predicted paths of the other vehicles, are passed to the Cost Function class to calculate the cost function for each state at each timestep.

The state with the minimum cost is selected as the desired state to realize.

Potential future improvements on this project would use the Vehicle class for the other vehicles to perform the collision detection as the current implementation relies on a vector of trajectories, and is a little confusing at times to debug.

### cost_functions.cpp / cost_functions.h

A Cost Function class was developed to calculate the cost for a given state and set of predicted trajectories.
The cost function header file contains the priority levels used in calculating how costly a proposed path is:
```
	// priority levels for costs
	int const COLLISION = pow(10, 6);
	int const DANGER = 4 * pow(10, 5);
	int const COMFORT = pow(10, 4);
	int const EFFICIENCY = pow(10, 3);
```

The total cost for a proposed path is made from the addition of four cost types:

#### Change Lane
Penalizes lane changes away from goal lane and rewards those towards goal lane. Uses the COMFORT priority level.

#### Inefficiency of Lane Change
Penalizes inefficient trajectories that result in slower speeds. Uses the EFFICIENCY priority level.

#### Collision
If there is an impending collision, increase cost using an exponential function, depending on timing. Uses the COLLISION priority level.

#### Buffer
Penalize trajectories that result in being closer than a DESIRED_BUFFER length to other vehicles in the proposed lane. Uses the DANGER priority level.

The total cost for the given state and trajectories is then passed back to the vehicle class for further processing.

## Challenges

### Latency

Latency was expected in the communication between the path planner and the simulator and was claimed to be handled internally in the simulator, however there was a major challenge with knowing exactly which timestep and prediction state / trajectory the project code was analyzing, compared to where the simulator thought all cars were. It was difficult to debug this project using command line code output due to the the mismatch in expected output and what the simulator was showing. It was determined that the simulator could be one or two timesteps out of sync with the path planner code, which had a noticeable affect on the collision detection for lane changes.

An attempt was made to try and debug using a graphical output to GnuPlot, which helped identify that there was an approximate 1 second, or 17 meter, offset of where the ego vehicle was showing, and what the sensor fusion data sent in for that timestep. This affected all aspects of the path planner including collision detection, spline interpolation. Adjusting the ego vehicles sensed `s` value in Frenet coordinates, by subtracting 17 m at each timestep, seemed to better match up the code data to the simulator in real time. Using GnuPlot was helpful, but limitied, because it introduced a significant lag in the processing time, which further increased latency.

A recommendation for future simulators would be to allow a graphical overlay or output of any image, text, or 2d graph that we feed into the simultor. The ability to see the data, as seen by simulator, in real time would have helped incredibly during debugging.

## Close-out

Overall this was the most challenging project to date, as it incorporated a significant number of new concepts, and a lot of code development and learning was left to the student. While this is useful in providing a strong sense of accomplishment once complete, the amount of time required to complete this project was far more than typically alotted for previous projects. It is obvious now that Behaviour Path Planning is one of the most complicated and challenging aspects of developing autonomous vehicles; from differences in timescales, to latency, to any number of places where bugs arise in logic and are very difficult to debug without a good simulator.
There are many opportunities for improvement in this code, but I am confident that this is a solveable engineering problem.

