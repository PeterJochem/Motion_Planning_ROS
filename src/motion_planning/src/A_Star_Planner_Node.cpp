/** @brief  
 */
#include "motion_planning/A_Star_Planner.hpp"
#include "motion_planning/plan.h"
#include "ros/ros.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>


class Planner { 

	public:
		bool startPlanning(motion_planning::plan::Request& request, motion_planning::plan::Response& response);
		void publishMarker(double x, double y);
		void processMap(const nav_msgs::OccupancyGrid& newMap);
		Planner();	
		A_Star_Planner my_A_Star_Planner = A_Star_Planner(nullptr, 1, 1, 1);		
	private:
		void plotPath(std::vector<std::tuple<double, double>>); 
		ros::NodeHandle n;		
		ros::Publisher marker_pub;
		ros::Publisher path_pub;
		// ros::Subscriber map_sub; // = n.subscribe("/map", 1, processMap);
		int markerCount;
		bool has_A_Map;
};		

/** @brief */
Planner::Planner() { 	
	
	marker_pub = n.advertise<visualization_msgs::Marker>("start_and_goal_locations", 1000);
	path_pub = n.advertise<visualization_msgs::MarkerArray>("/planned_path", 1);
	markerCount = 0;	
	//isPlanningNow = false;
	has_A_Map = false;
	//map_sub = n.subscribe("/map", 1, processMap);	
}

void Planner::processMap(const nav_msgs::OccupancyGrid& newMap) {
	// std::cout << std::endl << (int)map.data[0] << std::endl;
	
	std::vector<int8_t> mapCopy = newMap.data; 

	// Save the map in other data structure?
	// Planner will prevent the update if it is already planning but has not yet found a plan  
	my_A_Star_Planner.updateMap(mapCopy.data(), newMap.info.width, newMap.info.height, newMap.info.resolution);
	
	return;
}

bool Planner::startPlanning(motion_planning::plan::Request& request, motion_planning::plan::Response& response) {
	using namespace std;
	
	// A_Star_Planner::updateMap(int* map, int grid_width, int grid_height, double grid_resolution)	

	// setGoal(double start_map_x, double start_map_y, double goal_map_x, double goal_map_y)
	response.isPlanLegal = my_A_Star_Planner.setGoal(request.start_map_x, request.start_map_y, request.goal_map_x, request.goal_map_y);
		
	publishMarker(request.start_map_x, request.start_map_y);
        publishMarker(request.goal_map_x, request.goal_map_y);
	
	if (response.isPlanLegal) {
		vector<tuple<double, double>> waypoints = my_A_Star_Planner.plan();
		plotPath(waypoints);	
	}
	
	return true; 
}


/** @brief  */
void Planner::publishMarker(double x, double y) {
	// Draw a square in the start and goal locations in Gazebo and RVIZ     
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map"; // base_frame_id;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = markerCount;
        markerCount++;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5;

        marker.lifetime = ros::Duration(); // Marker will not vanish
        marker_pub.publish(marker);
}	

/** @brief */
void Planner::plotPath(std::vector<std::tuple<double, double>> path) { 
	
	visualization_msgs::MarkerArray markerarray;
	
	std::cout << "\n\n\n\n" << "The path length is " << path.size() << std::endl;

	for (int i = 0; i < path.size(); i++) {
		
		auto[nextX, nextY] = path[i];
		
		// Draw a square in the start and goal locations in Gazebo and RVIZ     
        	// Set our initial shape type to be a cube
        	uint32_t shape = visualization_msgs::Marker::CUBE;
        	visualization_msgs::Marker marker;

        	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
        	marker.header.frame_id = "map"; // base_frame_id;
        	marker.header.stamp = ros::Time::now();

        	// Set the namespace and id for this marker.  This serves to create a unique ID
        	// Any marker sent with the same namespace and id will overwrite the old one
        	marker.ns = "basic_shapes";
        	marker.id = markerCount;
        	markerCount++;

        	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        	marker.type = shape;
        	marker.action = visualization_msgs::Marker::ADD;

        	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        	marker.pose.position.x = nextX;
        	marker.pose.position.y = nextY;
        	marker.pose.position.z = 0;
        	marker.pose.orientation.x = 0.0;
        	marker.pose.orientation.y = 0.0;
        	marker.pose.orientation.z = 0.0;
        	marker.pose.orientation.w = 1.0;

        	// Set the scale of the marker -- 1x1x1 here means 1m on a side
        	marker.scale.x = 0.05;
        	marker.scale.y = 0.05;
        	marker.scale.z = 0.05;

        	// Set the color -- be sure to set alpha to something non-zero!
        	marker.color.r = 0.0f;
        	marker.color.g = 0.0f;
        	marker.color.b = 1.0f;
        	marker.color.a = 0.5;

	        marker.lifetime = ros::Duration(); // Marker will not vanish

		markerarray.markers.push_back(marker);
	}

	path_pub.publish(markerarray);
}



int main(int argc, char **argv) {

        ros::init(argc, argv, "motion_planning_node");
        ros::NodeHandle n;
	
	/*
        n.getParam("/rotational_vel_limit", max_rotation_speed);
        n.getParam("/trans_vel_limit", max_translational_speed);
        n.getParam("/k_p_trans", k_p_trans);
        n.getParam("/k_i_trans", k_i_trans);
        n.getParam("/k_p_rot", k_p_rot);
        n.getParam("/k_i_rot", k_i_rot);
        n.getParam("/linear_threshold", linear_threshold);
        n.getParam("/angular_threshold", angular_threshold);
	*/
		
        //A_Star_Planner(int* map, int height, int width, int resolution);
        //A_Star_Planner myPlanner = A_Star_Planner(nullptr, 1, 1, 100);
        //ros::Subscriber map_sub = n.subscribe("/map", 1, processMap);
	
	Planner myPlanner = Planner();
	
	ros::Subscriber map_sub = n.subscribe("/map", 1, &Planner::processMap, &myPlanner);	
	ros::ServiceServer service = n.advertiseService("StartPlanning", &Planner::startPlanning, &myPlanner);	
	
        while (ros::ok()) {
                ros::spinOnce();
                // Check if there are new map messages, if so, refresh data/look to refresh data
        }	
	

        return 0;
}

