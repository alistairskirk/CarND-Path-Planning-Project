#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_
#include <vector>
#include <math.h>
#include <functional>
#include "vehicle.h"

using namespace std;

class Cost_Functions
{
public:
	Cost_Functions();

	struct TrajectoryData {
		int proposed_lane;
		double avg_speed;
		double max_acceleration;
		double rms_acceleration;
		double closest_approach;
		double end_distance_to_goal;
		int end_lanes_from_goal;
		bool collides;
		int collides_at;
	};

	virtual ~Cost_Functions();

	// priority levels for costs
	int const COLLISION = pow(10, 6);
	int const DANGER = 3 * pow(10, 5);
	int const REACH_GOAL = pow(10, 5);
	int const COMFORT = pow(10, 4);
	int const EFFICIENCY = pow(10, 3);

	double const DESIRED_BUFFER = 1.5; // Number of timesteps

	int const PLANNING_HORIZON = 2;

	double calculate_cost(Vehicle vehicle, vector<Vehicle::snapshot> trajectory,
		map<int, vector < vector<double> > > predictions);
	
	double change_lane_cost(vector<Vehicle::snapshot> trajectory,
		map<int, vector < vector<double> > > predictions, TrajectoryData data);

	/* Would implement if you had a goal lane (like need to turn left or right at intersection or exit)
	double distance_from_goal_lane(vector<Vehicle::snapshot> trajectory,
		map<int, vector < vector<double> > > predictions, TrajectoryData data);
	*/

	double inefficiency_cost(Vehicle vehicle, vector<Vehicle::snapshot> trajectory,
		map<int, vector < vector<double> > > predictions, TrajectoryData data);

	double collision_cost(vector<Vehicle::snapshot> trajectory,
		map<int, vector < vector<double> > > predictions, TrajectoryData data);

	double buffer_cost(vector<Vehicle::snapshot> trajectory,
		map<int, vector < vector<double> > > predictions, TrajectoryData data);

	/* maybe implement this 
	double free_ahead_cost(vector<Vehicle::snapshot> trajectory,
		map<int, vector < vector<double> > > predictions, TrajectoryData data);
	*/

	TrajectoryData get_helper_data(Vehicle vehicle, vector<Vehicle::snapshot> trajectory,
		map<int, vector < vector<double> > > predictions);

	//bool check_collision(Vehicle::snapshot snap, prediction s_now);

	map<int, vector<vector<double> > > filter_predictions_by_lane(map<int, vector<vector<double> > > predictions, int lane);
	
};
#endif //COST_FUNCTIONS_H_
