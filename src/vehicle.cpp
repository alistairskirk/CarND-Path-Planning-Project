#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include "vehicle.h"
#include "cost_functions.h"

/**
* Initializes Vehicle
*/
Vehicle::Vehicle(int lane, double s, double v, double a) {

	this->lane = lane;
	this->s = s;
	this->v = v;
	this->a = a;
	state = "CS";
	max_acceleration = -1;	
}

Vehicle::~Vehicle() {}

void Vehicle::update_state(map<int, vector < vector<double> > > predictions) {
	ostringstream oss;
			
	/*
	Updates the "state" of the vehicle by assigning one of the
	following values to 'self.state':

	"KL" - Keep Lane
	- The vehicle will attempt to drive its target speed, unless there is
	traffic in front of it, in which case it will slow down.

	"LCL" or "LCR" - Lane Change Left / Right
	- The vehicle will IMMEDIATELY change lanes and then follow longitudinal
	behavior for the "KL" state in the new lane.

	"PLCL" or "PLCR" - Prepare for Lane Change Left / Right
	- The vehicle will find the nearest vehicle in the adjacent lane which is
	BEHIND itself and will adjust speed to try to get behind that vehicle.

	INPUTS
	- predictions
	A dictionary. The keys are ids of other vehicles and the values are arrays
	where each entry corresponds to the vehicle's predicted location at the
	corresponding timestep. The FIRST element in the array gives the vehicle's
	current position. Example (showing a car with id 3 moving at 2 m/s):

	{
	3 : [
	{"s" : 4, "lane": 0},
	{"s" : 6, "lane": 0},
	{"s" : 8, "lane": 0},
	{"s" : 10, "lane": 0},
	]
	}

	*/

	// Tie into trajectory planning and cost functions, set state

	// state = "KL"; // this is an example of how you change state.
	
	// Initialize the possible states
	vector< string> states = { "KL", "LCL", "LCR", "PLCL", "PLCR" };

	//remove left lane change if in the left hand lane
	if (this->lane == 0)
	{
		auto itr = std::find(states.begin(), states.end(), "LCL");
		if (itr != states.end()) states.erase(itr);
		//remove the prepLC too
		auto itr2 = std::find(states.begin(), states.end(), "PLCL");
		if (itr2 != states.end()) states.erase(itr2);
	}

	//remove right lane change if in the far right hand lane
	if (this->lane == (this->lanes_available - 1))
	{
		auto itr = std::find(states.begin(), states.end(), "LCR");
		if (itr != states.end()) states.erase(itr);
		//remove prepLC too
		auto itr2 = std::find(states.begin(), states.end(), "PLCR");
		if (itr2 != states.end()) states.erase(itr2);
	}

	// map of states and their costs
	map<string, double> costs;
	
	// Iterate through the available states and calculate other car trajectories and costs for that state
	for (auto &i : states) {
		//cout << "state: " << i << endl;
		//cout << "begin forloop" << endl;
		map<int, vector < vector<double> > > predictions_copy;
		predictions_copy.insert(predictions.begin(), predictions.end());
		double cost = 0.0;
		if (predictions_copy.size() > 0) // cost calculations will only work if predictions exist :(
		{
			vector<Vehicle::snapshot> trajectory = this->_trajectory_for_state(i, predictions_copy, 5);
			
			vector<double> traj_x;
			vector<double> traj_y;
			
			//check that the trajectories match the expected state
			if ((i == "LCR") | (i == "LCL")) {
				for (auto snap : trajectory) {
					//cout << "State: " << snap.state << " at s: " << snap.s<< " in lane: " << snap.lane << endl;				

				}
			}
			
			Cost_Functions cf;
			cost = cf.calculate_cost(*this, trajectory, predictions);
			dlog.append(cf.dlog);

		}
		
		//cout << "cost" << cost<< " for state " << i<< endl;
		costs.insert(pair<string, double>(i, cost));
		//cout << "done costs insert at i: " << i << endl;
		//cout << "states.size " << states.size() << endl;		
	}

	double mincost = pow(10,9);
	state = "KL";

	// Print costs map for debug
	oss << "costs map: " << endl;
	for (auto &i : costs) {
		if (i.second < mincost) {
			mincost = i.second;
			state = i.first;
		}
		oss << i.first << " " << i.second << endl;
		cout << i.first << " " << i.second << endl;
	}
	
	//For KL for debug
	//state = "KL";
			
	//Selected state with minimum cost
	oss << "chosen state: " << state << " at cost: " << mincost << endl;
	cout << "chosen state: " << state << " at cost: " << mincost << endl << endl;
	
	// if all costs are zero or equal, default is keep lane.
	
	//dlog.append(oss.str());
	oss.clear();

}

vector<Vehicle::snapshot> Vehicle::_trajectory_for_state(string state, map<int, vector < vector<double> > > &predictions, int horizon = 5) {
	// return a vector of snapshots for the given state, other vehicle predictions, and horizon
	//cout << " trajectory for state begin" << endl;
	// remember current state
	Vehicle::snapshot start_snap = this->create_snapshot();

	// pretend to be in new proposed state
	this->state = state;

	//init return vector and add the start state
	vector<Vehicle::snapshot> trajectory;
	trajectory.push_back(start_snap);

	/*for (auto car : predictions) 
	{
		std::cout << "prior to remove carid: " << car.first << endl;
		for (auto pred : car.second) {
			cout << "		lane: " << pred[0] << " at s : " << pred[1] << endl;
		}
	}*/

	//cout << "begin iterate" << endl;
	// interate to the horizon and add states to trajectory vector
	for (int i = 0; i < horizon; i++)
	{
		//need to remove first prediction for each vehicle
			
		this->lane = start_snap.lane;
		this->state = state;
		this->realize_state(predictions); 
		
		this->increment(1); 
		trajectory.push_back(this->create_snapshot());

		//cout << "begin remove first pred list of size: " << predictions.size() << " at horizon " << i << endl;
		for (auto car = predictions.begin(); car != predictions.end();)
		{
			car->second.erase(car->second.begin()); //Erasing the first prediction vector for this car
			car++;
		}

	}
	//cout << "call restore" << endl;
	//restore state from snapshot
	this->restore_state_from_snapshot(start_snap);
	//cout << "trajectory size:" << trajectory.size() << endl;
	return trajectory;
}

Vehicle::snapshot Vehicle::create_snapshot() {
	Vehicle::snapshot snap;
	snap.lane = this->lane;
	snap.s = this->s;
	snap.v = this->v;
	snap.a = this->a;
	//snap.state = "test";
	snap.state = this->state;
	return snap;
}

void Vehicle::restore_state_from_snapshot(Vehicle::snapshot snap) {
	//cout << "begin restore" << endl;
	this->lane = snap.lane;
	this->s = snap.s;
	this->v = snap.v;
	this->a = snap.a;
	this->state = snap.state;
	//cout << "done restore" << endl;
}


string Vehicle::display() {

	ostringstream oss;

	oss << "id:   " << this->id << "\n";
	oss << "s:    " << this->s << "\n";
	oss << "lane: " << this->lane << "\n";
	oss << "v:    " << this->v << "\n";
	oss << "a:    " << this->a << "\n";

	return oss.str();
}

void Vehicle::increment(int dt) {

	this->s += this->v * dt; 
	this->v += this->a * dt; 
	
}

vector<double> Vehicle::state_at(int t) {

	/*
	Predicts state of vehicle in t seconds (assuming constant acceleration)
	*/
	double s = this->s + this->v * t + this->a * t * t / 2;
	double v = this->v + this->a * t;
	return { (double)this->lane, s, v, this->a };
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
	Simple collision detection. Unused
	*/
	vector<double> check1 = state_at(at_time);
	vector<double> check2 = other.state_at(at_time);
	return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
	//Unused

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1;

	for (int t = 0; t < timesteps + 1; t++)
	{
		if (collides_with(other, t))
		{
			collider_temp.collision = true;
			collider_temp.time = t;
			return collider_temp;
		}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int, vector < vector<double> > > predictions) {

	/*
	Given a state, realize it by adjusting acceleration and lane.
	Note - lane changes happen instantaneously - need to tie into spline change
	*/
	string state = this->state;
	if (state.compare("CS") == 0)
	{
		realize_constant_speed();
	}
	else if (state.compare("KL") == 0)
	{
		realize_keep_lane(predictions);
	}
	else if (state.compare("LCL") == 0)
	{
		realize_lane_change(predictions, "L");
	}
	else if (state.compare("LCR") == 0)
	{
		realize_lane_change(predictions, "R");
	}
	else if (state.compare("PLCL") == 0)
	{
		realize_prep_lane_change(predictions, "L");
	}
	else if (state.compare("PLCR") == 0)
	{
		realize_prep_lane_change(predictions, "R");
	}

}

void Vehicle::realize_constant_speed() {
	a = 0.0;
}

double Vehicle::_max_accel_for_lane(map<int, vector<vector<double> > > predictions, int lane, int s) {

	double delta_v_til_target = target_speed - v;
	double max_acc = std::min(max_acceleration, delta_v_til_target);
	
	map<int, vector<vector<double> > >::iterator it = predictions.begin();
	vector<vector<vector<double> > > in_front;
	while (it != predictions.end())
	{

		int v_id = it->first;

		vector<vector<double> > v = it->second;

		//cout << "max acc. for lane, pred size: " << v.size() << endl;

		if ((v[0][0] == lane) && (v[0][1] > s))
		{
			in_front.push_back(v);

		}
		it++;
	}
	
	if (in_front.size() > 0)
	{
		double min_s = 1000.0;
		vector<vector<double>> leading = {};
		for (int i = 0; i < in_front.size(); i++)
		{
			if ((in_front[i][0][1] - s) < min_s)
			{
				min_s = (in_front[i][0][1] - s);
				leading = in_front[i];
			}
		}
		//cout << "leading size" << leading.size() << endl;
		if (leading.size() > 0) {
			double next_pos = leading[0][1];
			if (leading.size() > 1) {
				next_pos = leading[1][1];
			}
			double my_next = s + this->v;
			double separation_next = next_pos - my_next;
			double available_room = separation_next - preferred_buffer;
			max_acc = std::min(max_acc, available_room);
		}		
	}
	
	return max_acc;
	
}

void Vehicle::realize_keep_lane(map<int, vector< vector<double> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector< vector<double> > > predictions, string direction) {
	int delta = 1;
	//cout << "direction compare: " << direction.compare("L") << endl;
	if (direction.compare("L")==0)
	{
		delta = -1;
	}
	this->lane += delta;
	int lane = this->lane;
	int s = this->s;
	this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<double> > > predictions, string direction) {

	/*
	Need heirarchy logic here:
		1. Look at cars in goal lane.
		2. See if the gap is large enough, then pick that car to marge ahead (set goal_s).
		3. Adjust speed to match goal_s, or lead car minus a buffer.
		4. Check if Danger Close (pacing a car in current lane) 
			a. If new acceleration results in velocity > car ahead
			b. Then override acc to match car ahead	
	*/

	int delta = 1;
	if (direction.compare("L")==0)
	{
		delta = -1;
	}
	int current_lane = this->lane;
	int lane = current_lane + delta;

	map<int, vector<vector<double> > >::iterator it = predictions.begin(); // This is why predictions need to be removed before
	vector<vector<vector<double> > > at_behind;
	
	while (it != predictions.end())
	{
		int v_id = it->first;
		vector<vector<double> > v = it->second;

		//cout << "PLC vid " << v_id << " in lane: " << v[0][0] << " at s: " << v[0][1] << " ego_s: " << this->s << " ego_lane: " << this->lane << endl;

		if ((v[0][0] == lane) && (v[0][1] <= this->s))   // If car is in the state's lane, and behind ego, then add to behind list
		{
			at_behind.push_back(v);

		}
		it++;
	}
	if (at_behind.size() > 0)
	{

		double max_s = -1000.0;
		vector<vector<double> > nearest_behind = {};
		for (int i = 0; i < at_behind.size(); i++)
		{
			if ((at_behind[i][0][1]) > max_s)  // iterate through at behind list of cars in state lane, and find closest
			{
				max_s = at_behind[i][0][1];
				nearest_behind = at_behind[i];
			}
		}
		double target_vel = nearest_behind[1][1] - nearest_behind[0][1]; // delta_s in one timestep = velocity of that car
		//cout << "prep lane change " << direction << " to match target v: " << target_vel << endl;
		double delta_v = this->v - target_vel;	
		double delta_s = this->s - nearest_behind[0][1] - 3*L;
		//cout << "delta_v " << delta_v << " delta_s: " << delta_s << endl;
		if (delta_v != 0)
		{

			double time = -2 * delta_s / delta_v;
			double a;
			if (time == 0)
			{
				a = this->a;
			}
			else
			{
				a = delta_v / time;
			}
			if (a > this->max_acceleration)
			{
				a = this->max_acceleration;
			}
			if (a < -this->max_acceleration)
			{
				a = -this->max_acceleration;
			}
			//cout << "prep lane change acc before: " << this->a << endl;
			this->a = a;
			//cout << "prep lane change acc after: " << this->a << endl;
		}
		else
		{
			double my_min_acc = std::max((-1*(this->max_acceleration)), -delta_s);
			this->a = my_min_acc;
		}

	}

	// Need to watch for car ahead in current lane
	double current_a = this->_max_accel_for_lane(predictions, current_lane, this->s);
	if (current_a < this->a) { this->a = current_a; }

}

vector<vector<double> > Vehicle::generate_predictions(double horizon = 10.0) {

	vector<vector<double> > predictions;
	for (int i = 0; i < horizon; i++)
	{
		vector<double> check1 = state_at(i);		
		vector<double> lane_s = { check1[0], check1[1] };
		predictions.push_back(lane_s);
	}
	return predictions;

}