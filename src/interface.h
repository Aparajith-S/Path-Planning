#ifndef INTERFACE_H
#define INTERFACE_H
#include<vector>
namespace interfaces{
	struct ReadData 
	{
		double car_x;
		double car_y;
		double car_s;
		double car_d;
		double car_yaw;
		double car_speed;
	};
	struct mapData
	{
		std::vector<double> &map_waypoints_x;
		std::vector<double> &map_waypoints_y;
		std::vector<double> &map_waypoints_s;
		std::vector<double> &map_waypoints_dx;
		std::vector<double> &map_waypoints_dy;
	};
	using rawObjData = std::vector<double>;
	struct SnsFusionData 
	{
		std::vector<rawObjData>& raw_sensor_fusion;
	};

	struct PreviousOutputs
	{
		std::vector<double>& previous_path_x;
		std::vector<double>& previous_path_y;
		double end_path_s;
		double end_path_d;
	};

	struct input 
	{
		ReadData &currentData;
		PreviousOutputs &prevOp;
		SnsFusionData& snsFusion;
		mapData& mapdata;
	};

	struct output
	{
		std::vector<double> &next_x_vals;
		std::vector<double> &next_y_vals;
	};
}
#endif