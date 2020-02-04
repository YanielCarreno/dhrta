#include "dhrta/DHRTAStrategy.h"

/* implementation of rosplan_interface_mapping::DHRTAStrategy.h */
namespace DHRTA {

	/* constructor */
	DHRTAStrategy::DHRTAStrategy(ros::NodeHandle &nh){

		/* service call to implement the goal allocation */
		service_robots_distribution = nh.serviceClient<dhrta_msgs::RobotsDistribution>("/decentralised_robots_distribution");
		service_goal_allocator = nh.serviceClient<dhrta_msgs::GoalAllocation>("/decentralised_goal_allocation");
	}


	bool DHRTAStrategy::setupCoordinates(std::string goal_filename, std::string filename, std::string robot_filename) {

		/*
		* Regions Delimiter
		*/
		dhrta_msgs::RobotsDistribution srv_capability_analyser;
		srv_capability_analyser.request.robot_file = robot_filename;
		srv_capability_analyser.request.goals_file = goal_filename;

		service_robots_distribution.waitForExistence();
		if (service_robots_distribution.call(srv_capability_analyser)){
			ROS_INFO("DHRTA: Robots Distribution service is connected");
			bool capability_distribution = srv_capability_analyser.response.result;
			int robots_number = srv_capability_analyser.response.robot_no;
			if(capability_distribution == true){
				/*
				* Goal Allocation
				*/
				dhrta_msgs::GoalAllocation srv_regions_delimiter;
				srv_regions_delimiter.request.robots_number = robots_number;
				srv_regions_delimiter.request.waypoints_file = filename;
				srv_regions_delimiter.request.robot_file = robot_filename;
				srv_regions_delimiter.request.goals_file = goal_filename;
				srv_regions_delimiter.request.allocation_approach = "DHRTA";

				service_goal_allocator.waitForExistence();
				if (service_goal_allocator.call(srv_regions_delimiter)){
					ROS_INFO("DHRTA: Regions Delimiter service is connected");
					bool goals_allocation = srv_regions_delimiter.response.result;
					int constriants_no = srv_regions_delimiter.response.constraints_no;
					if(goals_allocation == true){
						ROS_INFO("DHRTA: Goal Distribution Finished ");
					}
					else{
						ROS_INFO("DHRTA: Goal Distribution Failed ");
					}
				}
				else{
					ROS_ERROR("DHRTA: Goal Allocation was not properly called");
				}
			}
			else{
				ROS_INFO("DHRTA: Robots Distribution didn't find the index distribution");
			}
		}
		else{
			ROS_ERROR("DHRTA: Robots Distribution was not properly called");
		}

	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		// setup ros
		ros::init(argc, argv, "dhrta_server");
		ros::NodeHandle nh("~");

		// params
		std::string filename("waypoints.txt");
		std::string goal_filename("goals.txt");
		std::string robot_filename("robots.txt");
		nh.param("waypoint_file", filename, filename);
		nh.param("goal_file", goal_filename, goal_filename);
		nh.param("robot_file", robot_filename, robot_filename);

		// initialisation
		DHRTA::DHRTAStrategy sms(nh);
		sms.setupCoordinates(goal_filename, filename, robot_filename);
		ROS_INFO("KCL: (DHRTAStrategy) Ready to receive.");
		ros::spin();
		return 0;
	}
