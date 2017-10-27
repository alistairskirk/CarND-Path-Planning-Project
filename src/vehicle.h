#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

	struct collider {

		bool collision; // is there a collision?
		int  time; // time collision happens

	};

	struct snapshot {
		//Create a snapshot ref point in time
		int lane;
		double s;
		double v;
		double a;
		string state;
	};

	int L = 1;

	int id = -1; // car id

	int preferred_buffer = 20; // impacts "keep lane" behavior.

	int lane;

	double s;

	double v;

	double a;

	double target_speed;

	int lanes_available;

	double max_acceleration;

	int goal_lane;

	int goal_s;

	string state;	

	/**
	* Constructor
	*/
	Vehicle(int lane, double s, double v, double a);

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	void update_state(map<int, vector <vector<double> > > predictions);

	vector<Vehicle::snapshot> _trajectory_for_state(string, map<int, vector < vector<double> > >, int);

	snapshot create_snapshot();

	void restore_state_from_snapshot(Vehicle::snapshot);
	
	string display();

	void increment(int dt);

	vector<double> state_at(int t);

	bool collides_with(Vehicle other, int at_time);

	collider will_collide_with(Vehicle other, int timesteps);

	void realize_state(map<int, vector < vector<double> > > predictions);

	void realize_constant_speed();

	double _max_accel_for_lane(map<int, vector<vector<double> > > predictions, int lane, int s);

	void realize_keep_lane(map<int, vector< vector<double> > > predictions);

	void realize_lane_change(map<int, vector< vector<double> > > predictions, string direction);

	void realize_prep_lane_change(map<int, vector< vector<double> > > predictions, string direction);

	vector<vector<double> > generate_predictions(double horizon);

};

#endif