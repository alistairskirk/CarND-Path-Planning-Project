#include <vector>
#include <math.h>
#include <functional>
#include "vehicle.h"
#include "cost_functions.h"
//#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communikation with Gnuplot

/**
* Cost_Functions Constructors and Destructors
*/

Cost_Functions::Cost_Functions() {}

Cost_Functions::~Cost_Functions() {}

// methods to calculate cost 

double Cost_Functions::calculate_cost(Vehicle vehicle, vector<Vehicle::snapshot> trajectory,
	map<int, vector < vector<double> > > predictions) {
	// Calculate and sum all the individual costs for this trajectory, given the other car predictions, and ego vehicle state
	
	double tot_cost = 0.0;
	Cost_Functions::TrajectoryData data = get_helper_data(vehicle, trajectory, predictions);
	double temp_cost = 0.0;
	temp_cost = change_lane_cost(trajectory, predictions, data);
	cout << "change lane cost:    " << temp_cost << endl;
	tot_cost += temp_cost;

	//tot_cost += distance_from_goal_lane(trajectory, predictions, data);

	temp_cost = inefficiency_cost(vehicle, trajectory, predictions, data);
	cout << "ineff. lane cost:    " << temp_cost << endl;
	tot_cost += temp_cost;

	temp_cost = collision_cost(trajectory, predictions, data);
	cout << "collision lane cost: " << temp_cost << endl;
	tot_cost += temp_cost;

	temp_cost = buffer_cost(trajectory, predictions, data);
	cout << "buffer cost:         " << temp_cost << endl;
	tot_cost += temp_cost;

	return tot_cost;
}

double Cost_Functions::change_lane_cost(vector<Vehicle::snapshot> trajectory,
	map<int, vector < vector<double> > > predictions, Cost_Functions::TrajectoryData data) {
	/* Penalizes lane changes away from goal lane and rewards those towards goal lane*/

	double this_cost = 0.0;
	int proposed_lanes = data.end_lanes_from_goal;
	int cur_lanes = trajectory[0].lane;

	if (proposed_lanes > cur_lanes) { this_cost = COMFORT; }
	if (proposed_lanes < cur_lanes) { this_cost = -COMFORT; }
	
	return this_cost;
}

/*
double distance_from_goal_lane(vector<Vehicle::snapshot> trajectory,
	map<int, vector < vector<double> > > predictions, Cost_Functions::TrajectoryData data) {
	// This cost function would be used for exiting or turning in a specified lane, not applicable for highway loop
	double this_cost = 0.0;

	return this_cost;
}
*/

double Cost_Functions::inefficiency_cost(Vehicle vehicle, vector<Vehicle::snapshot> trajectory,
	map<int, vector < vector<double> > > predictions, Cost_Functions::TrajectoryData data) {
	/*Penalizes inefficient moves that result in slower speeds*/
	double this_cost = 0.0;
	double speed = data.avg_speed;
	double target_speed = vehicle.target_speed;
	//cout << "target_speed and speed " << target_speed << " " << speed << endl;
	double pct = abs(target_speed - speed) / target_speed;
	double multiplier = pct*pct;
	this_cost = multiplier*EFFICIENCY;

	return this_cost;
}

double Cost_Functions::collision_cost(vector<Vehicle::snapshot> trajectory,
	map<int, vector < vector<double> > > predictions, Cost_Functions::TrajectoryData data) {
	/*If there is an impending collision, increase cost using an exponential function, depending on timing*/
	double this_cost = 0.0;
	//cout << "data.collides: " << data.collides << endl;
	if (data.collides) {
		double exponent = pow((double)data.collides_at, 2);
		double mult = exp(-exponent);
		this_cost = mult*COLLISION;
	}
	return this_cost;
}

double Cost_Functions::buffer_cost(vector<Vehicle::snapshot> trajectory,
	map<int, vector < vector<double> > > predictions, Cost_Functions::TrajectoryData data) {
	/**/
	double this_cost = 0.0;
	cout << "closest approach: " << data.closest_approach << endl;  
	if (data.closest_approach == 0.0) { this_cost = 10 * DANGER; }
	double t_away = data.closest_approach / data.avg_speed;
	cout << "t_away: " << t_away << endl;
	if (t_away > DESIRED_BUFFER) { this_cost = 0.0; }
	else {
		double mult = 1.0 - pow((t_away / DESIRED_BUFFER), 2);
		this_cost = mult*DANGER;
	}
	return this_cost;
}

/* maybe implement this
double free_ahead_cost(vector<Vehicle::snapshot> trajectory,
map<int, vector < vector<double> > > predictions, TrajectoryData data);
*/

Cost_Functions::TrajectoryData Cost_Functions::get_helper_data(Vehicle vehicle, vector<Vehicle::snapshot> trajectory,
	map<int, vector < vector<double> > > predictions) {
	// This function assembles the Trajectory Data struct from the given vehicle, trajectory, and predictions

	//Init variables
	Cost_Functions::TrajectoryData data;
	//cout << "trajectory size: " << trajectory.size()<<endl;
	Vehicle::snapshot current_snap = trajectory[0]; // Assumes trajectory has snapshots
	Vehicle::snapshot first = trajectory[1]; // need the second trajectory! It contains the simulated state
	Vehicle::snapshot last = trajectory.back();

	data.end_distance_to_goal = vehicle.goal_s - last.s; //not used yet
	data.end_lanes_from_goal = abs(vehicle.goal_lane - last.lane);

	//cout << "goal_lane and last lane: " << vehicle.goal_lane << " " << last.lane << endl;
	//cout << "end lanes from goal: " << data.end_lanes_from_goal << endl;

	double dt = trajectory.size()*.167; // Calibrated with set max speed in simulator, 0.167? should understand the physics of this.

	data.proposed_lane = first.lane;
	cout << "checking proposed lane: " << data.proposed_lane << " for state: " << first.state << endl;
	data.avg_speed = (last.s - current_snap.s) / dt;
	
	// Initialize collision variables
	data.closest_approach = 99999;
	data.collides = false;
	
	//Vehicle::snapshot last_snap = trajectory[0]; //not used
	
	// Get filtered predictions by lane
	map<int, vector<vector<double> > > filtered = filter_predictions_by_lane(predictions, data.proposed_lane);

	// Calculate collision and closest approach. This really needs to be improved	
	//for (int i = 1; i < PLANNING_HORIZON + 1; i++) {
		// use lane, s, v, a of each trajectory's snapshot	

		/*
		//cout << "filtered:" << endl;
		for (auto car : filtered) {	
			for (auto pred : car.second) {

				std::cout << "car_id: " << car.first << " in lane: " << pred[0] << " at s:  " << pred[1] << "\n";
			}			
		}
		*/

	for (auto car : filtered) {
		double s_previous;
		//cout << "car.second.size()" << car.second.size() << endl; //size of s of the current car, should be 10.
		for (int counter = 0; counter < PLANNING_HORIZON + 1;counter++) {
			//int counter = 0;
			vector<vector<double>> pred = car.second;  //assuming we are getting the prediction vector 
			//get first three predictions

			if (counter > 0) {
				//std::cout << "car_id: " << car.first << " in lane: " << pred[counter][0] << " at s:  " << pred[counter][1] << "\n";
				double s_now = pred[counter][1];
				//check if collision:
				//trajectory[i] is snapshot
				bool vehicle_collides = false;
				double _s = trajectory[counter].s;
				double _v = trajectory[counter].v;

				//cout << "s_now,s_prev,_s,_v: " << s_now << " " << s_previous << " " << _s << " " << _v << endl;

				double v_target = s_now - s_previous;
				if (s_previous < _s) {  //if he was behind ego
					if (s_now >= _s) { vehicle_collides = true; }  //then if he is now in front (collision happened)
				}
				if (s_previous > _s) {  //if he was in front
					if (s_now <= _s) { vehicle_collides = true; } //then if he is now behind, collision happened
				}
				if (s_previous == _s) {  //if you are in the same space he was
					if (v_target <= _v) { vehicle_collides = true; } //then if he was going slower than you, collision happened
				}

				//update data struct with collision
				if (vehicle_collides) {
					cout << "COLLISION at " << counter << " in lane: " << pred[counter][0] << " by carid: " << car.first << endl;
					data.collides = true;
					data.collides_at = counter;
				}				

				double dist = abs(s_now - _s);				
				
				if (dist < data.closest_approach) { 
					cout << "dist: " << dist << endl;
					data.closest_approach = dist;
				}
			}
			s_previous = pred[counter][1];			
		} //last_snap = trajectory[counter]; //increment last snapshot ? Not used
	}
		
	//cout <<"dcollides" << data.collides << endl;
	return data;
}

map<int, vector<vector<double> > > Cost_Functions::filter_predictions_by_lane(map<int, vector<vector<double> > > predictions, int lane) {
	map<int, vector<vector<double> > > filtered = {};
	for (auto car : predictions) {		
		if (car.second[0][0] == lane) {
			//std::cout <<"carid: "<< car.first << " in lane: " << car.second[0][0] << " at s:  " << car.second[0][1] << "\n";
			filtered[car.first] = car.second;
		}
	}
	return filtered;
}

