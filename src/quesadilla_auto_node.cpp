#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include "PlannerInput.pb.h"
#include "PlannerOutput.pb.h"

#include <quesadilla_auto_node/Planner_Input.h>
#include <quesadilla_auto_node/Planner_Output.h>

ros::NodeHandle* node;

quesadilla_auto_node::Planner_Input mROSPlannerInput;
quesadilla_auto_node::Planner_Output mROSPlannerOutput;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "quesadilla_auto_node");

	ros::NodeHandle n;

	node = &n;
	ros::Rate rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}