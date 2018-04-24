/**
 * @file /include/atest_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef atest_gui_QNODE_HPP_
#define atest_gui_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Bool.h>
#include <atest_gui/dynamixel_info.h>
#include "diana_msgs/BalanceParam.h"
#include "diana_msgs/BalanceParamWaist.h"
#include "diana_msgs/BalanceParamArm.h"
#include "diana_msgs/ForceTorque.h"
#include "diana_msgs/CenterChange.h"
#include "diana_msgs/FlagDataArray.h"
#include "diana_msgs/FlagDataTop.h"
#include "diana_foot_step_generator/FootStepCommand.h"
#include "diana_online_walking_module_msgs/SetBalanceParam.h"
#include "diana_online_walking_module_msgs/SetJointFeedBackGain.h"

#include <atest_gui/command.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#endif

#define all_DXL 23
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace atest_gui {

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread {
	Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void forceTorqueDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg);
	void currentArmStateMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void copFzMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void lLegPointXYZMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	void lLegPointRPYMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	void rLegPointXYZMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	void rLegPointRPYMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);

	void lCompensationXYZMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	void lCompensationRPYMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	void rCompensationXYZMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	void rCompensationRPYMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);


	void tfGyroValueMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);

	void currentFlagPosition1MsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	void currentFlagPosition2MsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);

	void initTopViewMsgCallback(const diana_msgs::FlagDataTop::ConstPtr& msg);
	void topViewRobotMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);



	// walking module
	ros::Publisher foot_step_command_pub;
	ros::ServiceClient set_balance_param_client;
	ros::ServiceClient joint_feedback_gain_client;

	ros::Publisher pattern_pub; // motion_pattern 입력
	ros::Publisher module_on_off; // 모듈 on off

	// sensor initialize button & completed signal detect
	ros::Publisher diana_ft_init_pub;
	ros::Subscriber diana_ft_init_done_sub;
	void dianaFtInitDoneMsgCallback(const std_msgs::Bool::ConstPtr& msg);
	bool ft_init_done_check;
	// base module //
	ros::Publisher init_pose_pub; //

	// off set module topic //
	ros::Publisher joint_select_pub;  // 오프셋 적용 하고자 하는 조인트
	ros::Publisher change_joint_value_pub; // 선택된 조인트 값 변경
	ros::ServiceClient read_joint_value_client;

	ros::Publisher save_onoff_pub; // motion on off
	ros::Publisher offset_joint_value_pub; // offset_joint_value

	// pose module topic //
	ros::Publisher desired_pose_pub;
	ros::Publisher desired_pose_waist_pub;
	ros::Publisher desired_pose_head_pub;
	ros::Publisher desired_pose_arm_pub;

	ros::Publisher gain_adjustment_pub;
	ros::Publisher final_gain_save_pub;
	ros::ServiceClient read_p_gain_value_client;
	ros::Subscriber current_arm_state_sub;

	// motion module balance on off topic //
	ros::Publisher diana_balance_parameter_pub; // 모듈 on off

	// center change topic //
	ros::Publisher center_change_pub;

	// sensor value
	ros::Subscriber imu_data_sub;
	ros::Subscriber diana_force_torque_data_sub;
	double currentGyroX_gui, currentGyroY_gui, currentGyroZ_gui;
	double current_orientationX_gui, current_orientationY_gui, current_orientationZ_gui;
	double currentForceX_l_gui, currentForceY_l_gui, currentForceZ_l_gui;
	double currentForceX_r_gui, currentForceY_r_gui, currentForceZ_r_gui;
	double currentTorqueX_l_gui, currentTorqueY_l_gui, currentTorqueZ_l_gui;
	double currentTorqueX_r_gui, currentTorqueY_r_gui, currentTorqueZ_r_gui;
	double l_arm_current_state_x, l_arm_current_state_y, l_arm_current_state_z;
	double r_arm_current_state_x, r_arm_current_state_y, r_arm_current_state_z;

	// motion module LEG
	ros::Subscriber cop_fz_sub;
	ros::Subscriber l_leg_point_xyz_sub;
	ros::Subscriber r_leg_point_xyz_sub;
	ros::Subscriber l_leg_point_rpy_sub;
	ros::Subscriber r_leg_point_rpy_sub;

	ros::Subscriber l_compensation_xyz_sub;
	ros::Subscriber r_compensation_xyz_sub;
	ros::Subscriber l_compensation_rpy_sub;
	ros::Subscriber r_compensation_rpy_sub;

	double current_cop_fz_x, current_cop_fz_y;
	double reference_cop_fz_x, reference_cop_fz_y;
	double lf_point_x, lf_point_y, lf_point_z, rf_point_x, rf_point_y, rf_point_z;

	double l_compensation_roll, l_compensation_pitch, l_compensation_yaw, r_compensation_roll, r_compensation_pitch, r_compensation_yaw;

	// upper module
	ros::Publisher diana_balance_parameter_waist_pub; // balance on off
	ros::Publisher head_balance_pub; // balance on off
	/// flag position
	ros::Subscriber current_flag_position1_sub;
	ros::Subscriber current_flag_position2_sub;
	double current_flag_position1[3];
	double current_flag_position2[3];

	// arm module
	ros::Publisher diana_balance_parameter_arm_pub;
	ros::Subscriber tf_gyro_value_sub;
	double tf_gyro_value_x, tf_gyro_value_y, tf_gyro_value_z;

	// decision_module
	ros::Publisher ready_check_pub;
	ros::Publisher update_pub;
	ros::Publisher init_check_pub;
	ros::Publisher mode_change_pub;

	ros::Subscriber init_top_view_sub;
	ros::Subscriber top_view_robot_sub;
	//geometry_msgs::Vector3 top_view_flag_position;
	double top_view_robot_x, top_view_robot_y;
	double in_flag_top_view_x[5], in_flag_top_view_y[5];
	double out_flag_top_view_x[5], out_flag_top_view_y[5];

	// remote time
	ros::Publisher remote_time_pub;


	Q_SIGNALS:
	void rosShutdown();

private:
	int init_argc;
	char** init_argv;


};

}  // namespace atest_gui

#endif /* atest_gui_QNODE_HPP_ */
