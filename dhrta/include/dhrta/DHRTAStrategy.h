#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include "diagnostic_msgs/KeyValue.h"
#include "geometry_msgs/PoseStamped.h"
#include "dhrta_msgs/RobotsDistribution.h"
#include "dhrta_msgs/GoalAllocation.h"

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <algorithm>
#include <iostream>
#include <stdlib.h>

namespace DHRTA {

	class DHRTAStrategy
	{

	private:

    // DHRTA Algorithm Services
		ros::ServiceClient service_robots_distribution;
		ros::ServiceClient service_goal_allocator;

	public:

		/* constructor */
		DHRTAStrategy(ros::NodeHandle &nh);

		/* service to (re)generate waypoints */
		bool setupCoordinates(std::string goal_filename, std::string filename, std::string robot_filename);
	};
}
