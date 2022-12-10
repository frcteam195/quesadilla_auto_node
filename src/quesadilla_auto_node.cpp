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

#include <rio_control_node/Robot_Status.h>
#include <quesadilla_auto_node/Planner_Input.h>
#include <quesadilla_auto_node/Planner_Output.h>
#include <quesadilla_auto_node/Quesadilla_Diagnostics.h>
#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define RECV_PORT     5803
#define SEND_PORT     5804
#define BUFSIZE 1500

ros::NodeHandle* node;

quesadilla_auto_node::Planner_Input::Request mROSPlannerInput;
quesadilla_auto_node::Planner_Output mROSPlannerOutput;
ros::Publisher mROSPlannerOutputPub;
ros::Publisher mQuesaDiagnosticsPub;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener;

std::recursive_mutex mThreadLock;

sockaddr_in mSIPAddr_in;
sockaddr_in mSIPAddr_out;

int fd_in;
int fd_out;
bool socketInitSuccess;
char outputBuffer[BUFSIZE];
static std::atomic_bool is_red_alliance {false};

static std::atomic<double> curr_x_in {0};
static std::atomic<double> curr_y_in {0};
static std::atomic<double> curr_yaw_deg {0};
static std::atomic_int curr_traj_id {-1};

bool socket_init()
{
	std::lock_guard<std::recursive_mutex> lock(mThreadLock);
    if (!socketInitSuccess)
    {
		memset(outputBuffer, 0, BUFSIZE);

		memset(&mSIPAddr_in, 0, sizeof(mSIPAddr_in));
		mSIPAddr_in.sin_family = AF_INET;
		mSIPAddr_in.sin_addr.s_addr = INADDR_ANY;
		mSIPAddr_in.sin_port = htons(RECV_PORT);

        fd_in = socket(AF_INET,SOCK_DGRAM,0);
        if(fd_in<0){
            ROS_ERROR("cannot open socket");
            return false;
        }


		// Bind the socket with the server address
		if ( bind(fd_in, (const struct sockaddr *)&mSIPAddr_in, sizeof(mSIPAddr_in)) < 0 )
		{
			ROS_ERROR("bind failed");
			ros::shutdown();
		}


		memset(&mSIPAddr_out, 0, sizeof(mSIPAddr_out));
		mSIPAddr_out.sin_family = AF_INET;
		mSIPAddr_out.sin_addr.s_addr = inet_addr("127.0.0.1");
		mSIPAddr_out.sin_port = htons(SEND_PORT);

        fd_out = socket(AF_INET,SOCK_DGRAM,0);
        if(fd_out<0){
            ROS_ERROR("cannot open socket");
            return false;
        }

        socketInitSuccess = true;
    }
    return socketInitSuccess;
}

bool ros_planner_input_callback(quesadilla_auto_node::Planner_Input::Request &request, quesadilla_auto_node::Planner_Input::Response &response)
{
	std::lock_guard<std::recursive_mutex> lock(mThreadLock);
	mROSPlannerInput.force_stop = request.force_stop;
	mROSPlannerInput.trajectory_id = request.trajectory_id;

	if (!mROSPlannerOutput.trajectory_active)
	{
		mROSPlannerInput.begin_trajectory = request.begin_trajectory;
	}
	else
	{
		mROSPlannerInput.begin_trajectory = false;
	}

	if (mROSPlannerInput.begin_trajectory || request.force_stop)
	{
		response.command_accepted = true;
	}
	else
	{
		response.command_accepted = false;
	}

	return response.command_accepted;
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

void robot_status_callback(const rio_control_node::Robot_Status &msg)
{
	is_red_alliance = msg.alliance == rio_control_node::Robot_Status::RED;
}

void ros_odom_callback(const nav_msgs::Odometry &msg)
{
	(void)msg;
	static ros::Time prevTime(0);
	try {
		ck::PlannerInput protoPlannerInput;
		protoPlannerInput.Clear();
		protoPlannerInput.set_timestamp(ros::Time::now().toSec());
		
		tf2::Stamped<tf2::Transform> localSideOdometryConverted;
		geometry_msgs::PoseStamped odomToRobot;
		odomToRobot.header.frame_id = "odom";
		odomToRobot.pose = msg.pose.pose;
		tfBuffer.transform(odomToRobot, localSideOdometryConverted, is_red_alliance ? "red_link" : "blue_link");
		ck::PlannerInput::Pose2d* pose2d = new ck::PlannerInput::Pose2d();
		pose2d->set_x(ck::math::meters_to_inches(localSideOdometryConverted.getOrigin().getX()));
		pose2d->set_y(ck::math::meters_to_inches(localSideOdometryConverted.getOrigin().getY()));
		pose2d->set_yaw(geometry::to_rotation(localSideOdometryConverted.getRotation()).yaw());
		protoPlannerInput.set_allocated_pose(pose2d);

		{
			std::lock_guard<std::recursive_mutex> lock(mThreadLock);
			protoPlannerInput.set_begin_trajectory(mROSPlannerInput.begin_trajectory);
			protoPlannerInput.set_force_stop(mROSPlannerInput.force_stop);
			protoPlannerInput.set_trajectory_id(mROSPlannerInput.trajectory_id);
			///////////////////////
			//Diag
			curr_traj_id = mROSPlannerInput.trajectory_id;
			///////////////////////
		}

		///////////////////////////////////////////////
		//Diag
		curr_x_in = ck::math::meters_to_inches(localSideOdometryConverted.getOrigin().getX());
		curr_y_in = ck::math::meters_to_inches(localSideOdometryConverted.getOrigin().getY());
		curr_yaw_deg = ck::math::rad2deg(geometry::to_rotation(localSideOdometryConverted.getRotation()).yaw());
		////////////////////////////////////////////////



		if (protoPlannerInput.SerializeToArray(outputBuffer, BUFSIZE))
		{
			ssize_t bSent = sendto(fd_out, outputBuffer, protoPlannerInput.ByteSizeLong(), 0, (struct sockaddr*)&mSIPAddr_out, sizeof(mSIPAddr_out));
			if (bSent < 0)
			{
				ROS_ERROR("Failed to send quesadilla proto planner input");
			}
		}
	}
	catch (...)
	{
		ROS_ERROR("Exception in odom callback for quesadilla");
	}
	prevTime = ros::Time::now();
}

void receive_data_thread()
{
	char buffer[BUFSIZE];
	memset(buffer, 0, BUFSIZE);

	while (ros::ok())
	{
		quesadilla_auto_node::Quesadilla_Diagnostics q_diag;
		{
			sockaddr recvFromAddr;
			socklen_t recvFromAddrSize;
			int numBytes = recvfrom(fd_in, &buffer, sizeof(buffer), 0, &recvFromAddr, &recvFromAddrSize);
			if (numBytes >= 0)
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
					mROSPlannerOutput.trajectory_completed = protoPlannerOutput.trajectory_completed();
					mROSPlannerOutput.trajectory_active = protoPlannerOutput.trajectory_active();
					mROSPlannerOutput.trajectory_id = protoPlannerOutput.trajectory_id();

					{
						std::lock_guard<std::recursive_mutex> lock(mThreadLock);						if (mROSPlannerOutput.trajectory_active)
						{
							mROSPlannerInput.begin_trajectory = false;
						}
					}

					{
						//Diagnostic Info
						q_diag.left_motor_output_rad_per_sec = protoPlannerOutput.left_motor_output_rad_per_sec();
						q_diag.left_motor_feedforward_voltage = protoPlannerOutput.left_motor_feedforward_voltage();
						q_diag.left_motor_accel_rad_per_sec2 = protoPlannerOutput.left_motor_accel_rad_per_sec2();
						q_diag.right_motor_output_rad_per_sec = protoPlannerOutput.right_motor_output_rad_per_sec();
						q_diag.right_motor_feedforward_voltage = protoPlannerOutput.right_motor_feedforward_voltage();
						q_diag.right_motor_accel_rad_per_sec2 = protoPlannerOutput.right_motor_accel_rad_per_sec2();
						q_diag.trajectory_completed = protoPlannerOutput.trajectory_completed();
						q_diag.trajectory_active = protoPlannerOutput.trajectory_active();
						q_diag.curr_x_in = curr_x_in;
						q_diag.curr_y_in = curr_y_in;
						q_diag.curr_yaw_deg = curr_yaw_deg;
						q_diag.curr_traj_id = curr_traj_id;
					}
				}
				else
				{
					setROSOutputZeros();
				}
			}
			else
			{
				ROS_ERROR("Socket Receive Quesadilla err: %d", numBytes);
			}
		}
		mROSPlannerOutputPub.publish(mROSPlannerOutput);
		mQuesaDiagnosticsPub.publish(q_diag);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "quesadilla_auto_node");
	ros::NodeHandle n;
	node = &n;

	tfListener = new tf2_ros::TransformListener(tfBuffer);

	ros::ServiceServer ros_planner_input_sub = node->advertiseService("quesadilla_planner_input", ros_planner_input_callback);
	ros::Subscriber ros_odom_sub = node->subscribe("/odometry/filtered", 1, ros_odom_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber robot_status_sub = node->subscribe("/RobotStatus", 1, robot_status_callback, ros::TransportHints().tcpNoDelay());
	mROSPlannerOutputPub = node->advertise<quesadilla_auto_node::Planner_Output>("/QuesadillaPlannerOutput", 1);
	mQuesaDiagnosticsPub = node->advertise<quesadilla_auto_node::Quesadilla_Diagnostics>("/QuesadillaDiagnostics", 1);

    while (!socket_init()){}

	std::thread recv_thread(receive_data_thread);

	ros::spin();

	return 0;
}