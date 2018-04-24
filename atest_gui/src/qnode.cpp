/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include "../include/atest_gui/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace atest_gui {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNode::QNode(int argc, char** argv ) :
																							   init_argc(argc),
																							   init_argv(argv)
{
	l_compensation_roll  = 0;
	l_compensation_pitch = 0;
	l_compensation_yaw   = 0;

	r_compensation_roll  = 0;
	r_compensation_pitch = 0;
	r_compensation_yaw   = 0;

	lf_point_x = 0;
	lf_point_y = 0;
	lf_point_z = 0;
	rf_point_x = 0;
	rf_point_y = 0;
	rf_point_z = 0;

	currentGyroX_gui = 0;
	currentGyroY_gui = 0;
	currentGyroZ_gui = 0;

	current_cop_fz_x = 0;
	current_cop_fz_y = 0;
	reference_cop_fz_x = 0;
	reference_cop_fz_y = 0;

	currentForceX_l_gui = 0;
	currentForceY_l_gui = 0;
	currentForceZ_l_gui = 0;
	currentForceX_r_gui = 0;
	currentForceY_r_gui = 0;
	currentForceZ_r_gui = 0;

	currentTorqueX_l_gui = 0;
	currentTorqueY_l_gui = 0;
	currentTorqueZ_l_gui = 0;
	currentTorqueX_r_gui = 0;
	currentTorqueY_r_gui = 0;
	currentTorqueZ_r_gui = 0;

	l_arm_current_state_x = 0;
	l_arm_current_state_y = 0;
	l_arm_current_state_z = 0;
	r_arm_current_state_x = 0;
	r_arm_current_state_y = 0;
	r_arm_current_state_z = 0;

	current_orientationX_gui = 0;
	current_orientationY_gui = 0;
	current_orientationZ_gui = 0;

	ft_init_done_check = 0;
	tf_gyro_value_x = 0;
	tf_gyro_value_y = 0;
	tf_gyro_value_z = 0;

	top_view_robot_x = 0;
	top_view_robot_y = 0;

	current_flag_position1[0] = 0;
	current_flag_position1[1] = 0;
	current_flag_position1[2] = 0;

	current_flag_position2[0] = 0;
	current_flag_position2[1] = 0;
	current_flag_position2[2] = 0;

	for(int num = 0; num < 5; num ++)
	{
		in_flag_top_view_x[num] = 0;
		in_flag_top_view_y[num] = 0;

		out_flag_top_view_x[num] = 0;
		out_flag_top_view_y[num] = 0;
	}
}

QNode::~QNode() {
	if(ros::isStarted()) {

		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}

	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"atest_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;


	start();

	// sensor initialize button & completed signal detect
	diana_ft_init_pub = n.advertise<std_msgs::Bool>("/diana/ft_init",10);
	diana_ft_init_done_sub = n.subscribe("/diana/ft_init_done", 10, &QNode::dianaFtInitDoneMsgCallback, this);

	// Add your ros communications here.
	module_on_off = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 10);
	// off set moudle topic definition //

	joint_select_pub = n.advertise<std_msgs::Int8>("/joint_select",10);
	change_joint_value_pub =n.advertise<std_msgs::Int16MultiArray>("/change_joint_value",10);

	read_joint_value_client = n.serviceClient<atest_gui::command>("/read_joint_value");

	save_onoff_pub = n.advertise<std_msgs::Bool>("/save_onoff",10);
	offset_joint_value_pub =n.advertise<std_msgs::Float64MultiArray>("/offset_joint_value",10);

	//base module //
	init_pose_pub = n.advertise<std_msgs::String>("/init_pose",10);

	//pose module //
	desired_pose_pub = n.advertise<std_msgs::Float64MultiArray>("/desired_pose_leg",10);
	desired_pose_waist_pub = n.advertise<std_msgs::Float64MultiArray>("/desired_pose_waist",10);
	desired_pose_head_pub = n.advertise<std_msgs::Float64MultiArray>("/desired_pose_head",10);
	desired_pose_arm_pub = n.advertise<std_msgs::Float64MultiArray>("/desired_pose_arm",10);
	current_arm_state_sub = n.subscribe("/current_arm_state", 10, &QNode::currentArmStateMsgCallback, this);


	gain_adjustment_pub = n.advertise<std_msgs::Int16MultiArray>("/gain_adjustment",10);
	final_gain_save_pub = n.advertise<std_msgs::Bool>("/final_gain_save",10);
	read_p_gain_value_client = n.serviceClient<atest_gui::command>("/read_p_gain_value");

	//motion_module //
	pattern_pub = n.advertise<std_msgs::Int32>("/motion_num",10);

	//balance on off
	diana_balance_parameter_pub = n.advertise<diana_msgs::BalanceParam>("/diana/balance_parameter",10);

	//center change
	center_change_pub = n.advertise<diana_msgs::CenterChange>("/diana/center_change",10);

	/////////////////////////////////////

	// sensor data //
	imu_data_sub = n.subscribe("/imu/data", 100, &QNode::imuDataMsgCallback, this);
	diana_force_torque_data_sub = n.subscribe("/diana/force_torque_data", 10, &QNode::forceTorqueDataMsgCallback, this);

	//motion module
	cop_fz_sub = n.subscribe("/cop_fz", 10, &QNode::copFzMsgCallback, this);
	l_leg_point_xyz_sub = n.subscribe("/l_leg_point_xyz", 10, &QNode::lLegPointXYZMsgCallback, this);
	r_leg_point_rpy_sub = n.subscribe("/l_leg_point_rpy", 10, &QNode::lLegPointRPYMsgCallback, this);

	r_leg_point_xyz_sub = n.subscribe("/r_leg_point_xyz", 10, &QNode::rLegPointXYZMsgCallback, this);
	r_leg_point_rpy_sub = n.subscribe("/r_leg_point_rpy", 10, &QNode::rLegPointRPYMsgCallback, this);

	l_compensation_xyz_sub = n.subscribe("/l_compensation_xyz", 10, &QNode::lCompensationXYZMsgCallback, this);
	l_compensation_rpy_sub = n.subscribe("/l_compensation_rpy", 10, &QNode::lCompensationRPYMsgCallback, this);
	r_compensation_xyz_sub = n.subscribe("/r_compensation_xyz", 10, &QNode::rCompensationXYZMsgCallback, this);
	r_compensation_rpy_sub = n.subscribe("/r_compensation_rpy", 10, &QNode::rCompensationRPYMsgCallback, this);

	//upper balance module
	diana_balance_parameter_waist_pub = n.advertise<diana_msgs::BalanceParamWaist>("/diana/balance_parameter_waist",10);
	current_flag_position1_sub = n.subscribe("/current_flag_position1", 10, &QNode::currentFlagPosition1MsgCallback, this);
	current_flag_position2_sub = n.subscribe("/current_flag_position2", 10, &QNode::currentFlagPosition2MsgCallback, this);

	// arm balance module
	diana_balance_parameter_arm_pub = n.advertise<diana_msgs::BalanceParamArm>("/diana/balance_parameter_arm",10);
	head_balance_pub = n.advertise<std_msgs::Bool>("/head_balance",10);
	tf_gyro_value_sub = n.subscribe("/tf_gyro_value", 10, &QNode::tfGyroValueMsgCallback, this);


	// decision_module
	ready_check_pub = n.advertise<std_msgs::Bool>("/ready_check",10);
	update_pub      = n.advertise<std_msgs::Bool>("/update",10);
	mode_change_pub = n.advertise<std_msgs::Bool>("/mode_change",10);
	init_check_pub  = n.advertise<std_msgs::Bool>("/init_check",10);

	init_top_view_sub    = n.subscribe("/init_top_view", 10, &QNode::initTopViewMsgCallback, this);
	top_view_robot_sub    = n.subscribe("/top_view_robot", 10, &QNode::topViewRobotMsgCallback, this);

	remote_time_pub      = n.advertise<std_msgs::Bool>("/remote_time",10);

	//walking module

	foot_step_command_pub = n.advertise<diana_foot_step_generator::FootStepCommand>("/heroehs/diana_foot_step_generator/walking_command",10);
	set_balance_param_client =  n.serviceClient<diana_online_walking_module_msgs::SetBalanceParam>("/heroehs/online_walking/set_balance_param");
	joint_feedback_gain_client = n.serviceClient<diana_online_walking_module_msgs::SetJointFeedBackGain>("/heroehs/online_walking/joint_feedback_gain");


	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
	return true;
}
void QNode::run() {
	while ( ros::ok() ) {

		ros::spinOnce();

	}
}

void QNode::topViewRobotMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	top_view_robot_x = msg->x;
	top_view_robot_y = msg->y;
}
void QNode::initTopViewMsgCallback(const diana_msgs::FlagDataTop::ConstPtr& msg)
{

	for(int num = 0; num < 5; num ++)
	{
		in_flag_top_view_x[num] = msg->flag_in_data_m_x[num];
		in_flag_top_view_y[num] = msg->flag_in_data_m_y[num];

		out_flag_top_view_x[num] = msg->flag_out_data_m_x[num];
		out_flag_top_view_y[num] = msg->flag_out_data_m_y[num];
	}
}

void QNode::dianaFtInitDoneMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ft_init_done_check = msg->data;
}
void QNode::currentFlagPosition1MsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	current_flag_position1[0] = msg->x;
	current_flag_position1[1] = msg->y;
	current_flag_position1[2] = msg->z;
}
void QNode::currentFlagPosition2MsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	current_flag_position2[0] = msg->x;
	current_flag_position2[1] = msg->y;
	current_flag_position2[2] = msg->z;
}
void QNode::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	currentGyroX_gui = (double) msg->angular_velocity.x;
	currentGyroY_gui = (double) -msg->angular_velocity.y;
	currentGyroZ_gui = (double) msg->angular_velocity.z;

	current_orientationX_gui  = (double) msg->orientation.x;
	current_orientationY_gui  = (double) msg->orientation.y;
	current_orientationZ_gui  = (double) msg->orientation.z;
}

void QNode::forceTorqueDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg)
{
	currentForceX_l_gui = (double) msg->force_x_raw_l;
	currentForceY_l_gui = (double) msg->force_y_raw_l;
	currentForceZ_l_gui = (double) msg->force_z_raw_l;

	currentForceX_r_gui = (double) msg->force_x_raw_r;
	currentForceY_r_gui = (double) msg->force_y_raw_r;
	currentForceZ_r_gui = (double) msg->force_z_raw_r;

	currentTorqueX_l_gui = (double) msg->torque_x_raw_l;
	currentTorqueY_l_gui = (double) msg->torque_y_raw_l;
	currentTorqueZ_l_gui = (double) msg->torque_z_raw_l;

	currentTorqueX_r_gui = (double) msg->torque_x_raw_r;
	currentTorqueY_r_gui = (double) msg->torque_y_raw_r;
	currentTorqueZ_r_gui = (double) msg->torque_z_raw_r;
}
void QNode::currentArmStateMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	l_arm_current_state_x = (double) msg->data[0];
	l_arm_current_state_y = (double) msg->data[1];
	l_arm_current_state_z = (double) msg->data[2];

	r_arm_current_state_x = (double) msg->data[3];
	r_arm_current_state_y = (double) msg->data[4];
	r_arm_current_state_z = (double) msg->data[5];
}
void QNode::copFzMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	current_cop_fz_x   = (double) msg->data[0];
	current_cop_fz_y   = (double) msg->data[1];
	reference_cop_fz_x = (double) msg->data[2];
	reference_cop_fz_y = (double) msg->data[3];
}
void QNode::lLegPointXYZMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	lf_point_x = (double) msg->x;
	lf_point_y = (double) msg->y;
	lf_point_z = (double) msg->z;
}
void QNode::lLegPointRPYMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	return;
}
void QNode::rLegPointXYZMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	rf_point_x = (double) msg->x;
	rf_point_y = (double) msg->y;
	rf_point_z = (double) msg->z;
}
void QNode::rLegPointRPYMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	return;
}
void QNode::lCompensationXYZMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	return;
}
void QNode::lCompensationRPYMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	l_compensation_roll  = (double) msg->x;
	l_compensation_pitch = (double) msg->y;
	l_compensation_yaw   = (double) msg->z;
}
void QNode::rCompensationXYZMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	return;
}
void QNode::rCompensationRPYMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	r_compensation_roll  = (double) msg->x;
	r_compensation_pitch = (double) msg->y;
	r_compensation_yaw   = (double) msg->z;
}
void QNode::tfGyroValueMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	tf_gyro_value_x = (double) msg->x;
	tf_gyro_value_y = (double) msg->y;
	tf_gyro_value_z = (double) msg->z;

}
}  // namespace atest_gui
