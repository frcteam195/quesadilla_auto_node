#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <atomic>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "PlannerInput.pb.h"
#include "PlannerOutput.pb.h"

#include <quesadilla_auto_node/Planner_Input.h>
#include <quesadilla_auto_node/Planner_Output.h>

#define PORT     5803
#define BUFSIZE 1500

ros::NodeHandle* node;

quesadilla_auto_node::Planner_Input mROSPlannerInput;
quesadilla_auto_node::Planner_Output mROSPlannerOutput;
ros::Publisher mROSPlannerOutputPub;

ck::PlannerInput mProtoPlannerInput;
ck::PlannerOutput mProtoPlannerOutput;

std::recursive_mutex mThreadLock;

int fd;
bool socketInitSuccess;

bool socket_init()
{
	std::lock_guard<std::recursive_mutex> lock(mThreadLock);
    if (!socketInitSuccess)
    {
        fd = socket(AF_INET,SOCK_DGRAM,0);
        if(fd<0){
            ROS_ERROR("cannot open socket");
            return false;
        }
        socketInitSuccess = true;
    }
    return socketInitSuccess;
}

void ros_planner_input_callback(const quesadilla_auto_node::Planner_Input &msg)
{
	std::lock_guard<std::recursive_mutex> lock(mThreadLock);
	mROSPlannerInput = msg;
}

void setROSOutputZeros()
{
	mROSPlannerOutput.leftMotorOutputRadPerSec = 0;
	mROSPlannerOutput.leftMotorFeedForwardVoltage = 0;
	mROSPlannerOutput.leftMotorAccelRadPerSec2 = 0;
	mROSPlannerOutput.rightMotorOutputRadPerSec = 0;
	mROSPlannerOutput.rightMotorFeedForwardVoltage = 0;
	mROSPlannerOutput.rightMotorAccelRadPerSec2 = 0;
	mROSPlannerOutput.trajectoryActive = false;
	mROSPlannerOutput.trajectoryCompleted = false;
}

void send_planner_input()
{
	char buffer[BUFSIZE];
	memset(buffer, 0, BUFSIZE);
	{
		std::lock_guard<std::recursive_mutex> lock(mThreadLock);
		while (!socket_init()){}
	}

	ros::Rate rate(100);
	while (ros::ok())
	{
		{
			std::lock_guard<std::recursive_mutex> lock(mThreadLock);
			if (!socketInitSuccess)
			{
				socket_init();
			}
			
		}
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "quesadilla_auto_node");
	ros::NodeHandle n;
	node = &n;

	ros::Subscriber ros_planner_input_sub = node->subscribe("/QuesadillaPlannerInput", 1, ros_planner_input_callback);
	mROSPlannerOutputPub = node->advertise<quesadilla_auto_node::Planner_Output>("/QuesadillaPlannerOutput", 1);

	char buffer[BUFSIZE];
	memset(buffer, 0, BUFSIZE);

    while (!socket_init()){}

	sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

	// Bind the socket with the server address
    if ( bind(fd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 )
    {
        ROS_ERROR("bind failed");
		ros::shutdown();
    }

	std::thread mPlannerInputSendThread(send_planner_input);

	ros::Rate rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		{
			std::lock_guard<std::recursive_mutex> lock(mThreadLock);
			sockaddr recvFromAddr;
			socklen_t recvFromAddrSize;
			int numBytes = recvfrom(fd, &buffer, sizeof(buffer), MSG_WAITALL, &recvFromAddr, &recvFromAddrSize);
		}
		mROSPlannerOutputPub.publish(mROSPlannerOutput);
		rate.sleep();
	}

	mPlannerInputSendThread.join();

	return 0;
}