#ifndef INTERFACE_H
#define INTERFACE_H
#include<vector>
namespace interfaces{
	struct EgoData 
	{
		double car_x;
		double car_y;
		double car_s;
		double car_d;
		double car_yaw;
		double car_speed;
		uint8_t car_lane;
		bool car_danger;
		double car_target_speed;
	};
	struct ObjData 
	{
		double car_x;
		double car_y;
		double car_s;
		double car_d;
		double car_yaw;
		double car_speed;
		int car_lane;
	};
	struct mapData
	{
		std::vector<double> map_waypoints_x;
		std::vector<double> map_waypoints_y;
		std::vector<double> map_waypoints_s;
		std::vector<double> map_waypoints_dx;
		std::vector<double> map_waypoints_dy;
	};
	using rawObjData = std::vector<double>;
	struct SnsFusionData 
	{
		std::vector<rawObjData> raw_sensor_fusion;
	};

	struct PreviousOutputs
	{
		std::vector<double> previous_path_x;
		std::vector<double> previous_path_y;
		double end_path_s;
		double end_path_d;
	};

	struct input 
	{
		EgoData currentData;
		PreviousOutputs prevOp;
		SnsFusionData snsFusion;
	};

	struct output
	{
		std::vector<double> next_x_vals;
		std::vector<double> next_y_vals;
	};
	struct dfrenet
	{	dfrenet(double v=0.0,double vd=0.0,double vdd=0.0): value(v),value_dot(vd),value_ddot(vdd){}
		~dfrenet(){}
		double value;
		double value_dot;
		double value_ddot;
	};
	struct TrajectoryFrenet
	{
		std::vector<dfrenet> s_vals;
		std::vector<dfrenet> d_vals;
	};

	struct TrajectoryCart
	{
		std::vector<double> x_vals;
		std::vector<double> y_vals;
	};
	struct TrajectorySet
	{
		TrajectoryFrenet frenet;
		TrajectoryCart cart;
	};

	struct PrevTrajectory
	{
		TrajectoryCart Prev_cart;
		TrajectoryFrenet Prev_frenet;
		int prev_reused_xy;
	};

using coefficients = std::vector<double>;
struct chosenTrajectory 
{
    coefficients traj_s;
    coefficients traj_d;
    double traj_T;
};
}
#endif