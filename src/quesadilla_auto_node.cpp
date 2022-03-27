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
#include <nav_msgs/Odometry.h>
#include <ck_utilities/CKMath.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define PORT     5803
#define BUFSIZE 1500

ros::NodeHandle* node;

quesadilla_auto_node::Planner_Input mROSPlannerInput;
quesadilla_auto_node::Planner_Output mROSPlannerOutput;
ros::Publisher mROSPlannerOutputPub;

std::recursive_mutex mThreadLock;

sockaddr_in mSIPAddr;

int fd;
bool socketInitSuccess;
char outputBuffer[BUFSIZE];

bool socket_init()
{
	std::lock_guard<std::recursive_mutex> lock(mThreadLock);
    if (!socketInitSuccess)
    {
		memset(outputBuffer, 0, BUFSIZE);

		memset(&mSIPAddr, 0, sizeof(mSIPAddr));
		mSIPAddr.sin_family = AF_INET;
		mSIPAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
		mSIPAddr.sin_port = htons(PORT);

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
	mROSPlannerOutput.left_motor_output_rad_per_sec = 0;
	mROSPlannerOutput.left_motor_feedforward_voltage = 0;
	mROSPlannerOutput.left_motor_accel_rad_per_sec2 = 0;
	mROSPlannerOutput.right_motor_output_rad_per_sec = 0;
	mROSPlannerOutput.right_motor_feedforward_voltage = 0;
	mROSPlannerOutput.right_motor_accel_rad_per_sec2 = 0;
	mROSPlannerOutput.trajectory_active = false;
	mROSPlannerOutput.trajectory_completed = false;
}

void ros_odom_callback(const nav_msgs::Odometry &msg)
{
	ck::PlannerInput protoPlannerInput;
	protoPlannerInput.Clear();
	protoPlannerInput.set_timestamp(ros::Time::now().toSec());
	ck::PlannerInput::Pose2d* pose2d = new ck::PlannerInput::Pose2d();
	pose2d->set_x(ck::math::meters_to_inches(msg.pose.pose.position.x));
	pose2d->set_y(ck::math::meters_to_inches(msg.pose.pose.position.y));
	tf2::Quaternion rotation(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf2::Matrix3x3 q(rotation);
	double r, p, y;
	q.getRPY(r, p, y);
	pose2d->set_yaw(y);
	protoPlannerInput.set_allocated_pose(pose2d);
	{
		std::lock_guard<std::recursive_mutex> lock(mThreadLock);
		protoPlannerInput.set_begin_trajectory(mROSPlannerInput.begin_trajectory);
		protoPlannerInput.set_force_stop(mROSPlannerInput.force_stop);
		protoPlannerInput.set_trajectory_id(mROSPlannerInput.trajectory_id);
	}

	if (protoPlannerInput.SerializeToArray(outputBuffer, BUFSIZE))
	{
		ssize_t bSent = sendto(fd, outputBuffer, protoPlannerInput.ByteSizeLong(), 0, (struct sockaddr*)&mSIPAddr, sizeof(mSIPAddr));
		if (bSent < 0)
		{
			ROS_ERROR("Failed to send quesadilla proto planner input");
		}
	}
}

void receive_data_thread()
{
	char buffer[BUFSIZE];
	memset(buffer, 0, BUFSIZE);

	while (ros::ok())
	{
		{
			sockaddr recvFromAddr;
			socklen_t recvFromAddrSize;
			int numBytes = recvfrom(fd, &buffer, sizeof(buffer), MSG_WAITALL, &recvFromAddr, &recvFromAddrSize);
			if (numBytes > 0)
			{
				ck::PlannerOutput protoPlannerOutput;
				if(protoPlannerOutput.ParseFromArray(buffer, numBytes))
				{
					mROSPlannerOutput.left_motor_output_rad_per_sec = protoPlannerOutput.left_motor_output_rad_per_sec();
					mROSPlannerOutput.left_motor_feedforward_voltage = protoPlannerOutput.left_motor_feedforward_voltage();
					mROSPlannerOutput.left_motor_accel_rad_per_sec2 = protoPlannerOutput.left_motor_accel_rad_per_sec2();
					mROSPlannerOutput.right_motor_output_rad_per_sec = protoPlannerOutput.right_motor_output_rad_per_sec();
					mROSPlannerOutput.right_motor_feedforward_voltage = protoPlannerOutput.right_motor_feedforward_voltage();
					mROSPlannerOutput.right_motor_accel_rad_per_sec2 = protoPlannerOutput.right_motor_accel_rad_per_sec2();
					mROSPlannerOutput.trajectory_active = protoPlannerOutput.trajectory_active();
					mROSPlannerOutput.trajectory_completed = protoPlannerOutput.trajectory_completed();
				}
				else
				{
					setROSOutputZeros();
				}
			}
		}
		mROSPlannerOutputPub.publish(mROSPlannerOutput);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "quesadilla_auto_node");
	ros::NodeHandle n;
	node = &n;

	ros::Subscriber ros_planner_input_sub = node->subscribe("/QuesadillaPlannerInput", 1, ros_planner_input_callback);
	ros::Subscriber ros_odom_sub = node->subscribe("/odometry/filtered", 1, ros_odom_callback);
	mROSPlannerOutputPub = node->advertise<quesadilla_auto_node::Planner_Output>("/QuesadillaPlannerOutput", 1);

    while (!socket_init()){}

	// Bind the socket with the server address
    if ( bind(fd, (const struct sockaddr *)&mSIPAddr, sizeof(mSIPAddr)) < 0 )
    {
        ROS_ERROR("bind failed");
		ros::shutdown();
    }

	std::thread recv_thread(receive_data_thread);

	ros::spin();

	return 0;
}