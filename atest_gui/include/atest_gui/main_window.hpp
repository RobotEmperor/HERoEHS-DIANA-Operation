/**
 * @file /include/atest_gui/main_window.hpp
 *
 * @brief Qt based gui for atest_gui.
 *
 * @date November 2010
 **/
#ifndef atest_gui_MAIN_WINDOW_H
#define atest_gui_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <QKeyEvent>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <vector>
#include <math.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <QString>
#include "diana_msgs/BalanceParam.h"
#include "diana_msgs/BalanceParamWaist.h"
#include "diana_msgs/BalanceParamArm.h"
#include "diana_msgs/FlagDataArray.h"
#include "diana_foot_step_generator/FootStepCommand.h"
#include "diana_online_walking_module_msgs/SetBalanceParam.h"
#include "diana_online_walking_module_msgs/SetJointFeedBackGain.h"
#endif

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace atest_gui {

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	public Q_SLOTS:
	void graph_draw(QCustomPlot *ui_graph, const QString title, const QString unit, int min_value, int max_value, int tick_count);
	void graph_draw_update(QCustomPlot *ui_graph, double valueX, double valueY, double valueZ);
	void graph_draw_clean(QCustomPlot *ui_graph);

	void graph_draw_none_line(QCustomPlot *ui_graph, const QString title, const QString unit, double min_value_x, double max_value_x, double min_value_y, double max_value_y, int tick_count);
	void graph_draw_update_none_line(QCustomPlot *ui_graph, double cur_value1, double cur_value2, double ref_value1, double ref_value2, double lf_value1, double lf_value2, double rf_value1, double rf_value2);

	void graph_draw_none_line_flag(QCustomPlot *ui_graph, const QString title, const QString unit, double min_value_x, double max_value_x, double min_value_y, double max_value_y, int tick_count);
	void graph_draw_update_none_line_flag(QCustomPlot *ui_graph, double robot_x, double robot_y, double flag1_x, double flag1_y, double flag2_x, double flag2_y);


	void graph_draw_top_view(QCustomPlot *ui_graph, const QString title, const QString unit, double min_value_x, double max_value_x, double min_value_y, double max_value_y, int tick_count);
	void graph_draw_update_top_view(QCustomPlot *ui_graph, double robot_x, double robot_y , double in_top_veiw_x[5], double in_top_veiw_y[5], double out_top_veiw_x[5], double out_top_veiw_y[5]);


	void realtimeDataSlot();

	//slot
	// sensor initialize button
	void on_initialize_ft_sensor_button_clicked();
	void on_graph_stop_button_1_clicked();
	void on_graph_start_button_1_clicked();

	//offset module ///////////////////////////////////////////////////////////////////
	void on_off_module_2_clicked();
	void on_offset_module_clicked();

	void on_read_data_clicked();
	void on_plus_button_clicked();
	void on_minus_button_clicked();
	void on_read_data_one_clicked();

	void on_save_onoff_clicked();
	void on_initial_button_clicked();
	void on_send_offset_value_clicked();
	void on_init_offset_pose_button_clicked();

	//base module ///////////////////////////////////////////////////////////////////
	void on_base_module_1_clicked();
	void on_init_module_call_clicked();
	void on_off_module_3_clicked();
	// control GUI slot
	void on_pflug_bogen_button_clicked();
	void on_parallel_button_clicked();
	void on_carving_button_clicked();

	void on_center_change_button_clicked();
	void on_edge_change_button_clicked();
	// control
	void center_change(int k);
	void edge_change(int k);
	//////////////////////////////////////////////////////////////////////////////////////

	// pose module ///////////////////////////////////////////////////////////////////
	void on_off_module_clicked();
	void on_pose_module_clicked();
	void on_initialize_module_2_clicked();
	// pose PID gain gui slot
	void parse_gain_data();

	void on_l_hip_pitch_button_clicked();
	void on_l_hip_roll_button_clicked();
	void on_l_hip_yaw_button_clicked();

	void on_l_knee_pitch_button_clicked();
	void on_l_ankle_pitch_button_clicked();
	void on_l_ankle_roll_button_clicked();

	void on_r_hip_pitch_button_clicked();
	void on_r_hip_roll_button_clicked();
	void on_r_hip_yaw_button_clicked();

	void on_r_knee_pitch_button_clicked();
	void on_r_ankle_pitch_button_clicked();
	void on_r_ankle_roll_button_clicked();

	void on_head_yaw_button_clicked();
	void on_head_pitch_button_clicked();
	void on_head_roll_button_clicked();

	void on_waist_roll_button_clicked();
	void on_waist_yaw_button_clicked();

	void on_l_shoulder_pitch_button_clicked();
	void on_l_shoulder_roll_button_clicked();
	void on_l_elbow_pitch_button_clicked();
	void on_r_shoulder_pitch_button_clicked();
	void on_r_shoulder_roll_button_clicked();
	void on_r_elbow_pitch_button_clicked();

	void on_initial_p_gain_load_button_clicked();
	void on_final_gain_save_button_clicked();
	void on_gain_adjustment_button_clicked();

	// pose generator gui slot
	void on_change_kinematics_1_1_clicked(); // leg 12 variables change
	void on_change_kinematics_2_clicked(); // 2 pose

	void on_waist_change_button_clicked();
	void on_head_change_button_clicked();
	//////////////////////////////////////////////////////////////////////////////////////
	// motion module ///////////////////////////////////////////////////////////////////
	void on_motion_module_clicked();
	void on_balance_on_clicked();
	void on_balance_off_clicked();
	void on_cop_control_leg_on_clicked();
	void on_cop_control_leg_off_clicked();
	void on_off_module_4_clicked();
	// upper module ///////////////////////////////////////////////////////////////////////
	void on_upper_body_module_button_clicked();
	void on_balance_on_waist_button_clicked();
	void on_balance_off_waist_button_clicked();
	void on_balance_on_head_button_clicked();
	void on_balance_off_head_button_clicked();
	void on_cop_control_waist_on_clicked();
	void on_cop_control_waist_off_clicked();
	void parse_gain_waist_data();

	// arm module ///////////////////////////////////////////////////////////////////////
	void on_arm_module_button_clicked();
	void on_arm_change_end_effector_button_clicked();
	void on_arm_change_joint_button_clicked();

	void on_balance_on_arm_button_clicked();
	void on_balance_off_arm_button_clicked();
	void on_cop_control_arm_on_clicked();
	void on_cop_control_arm_off_clicked();
	void parse_gain_arm_data();

	//common button
	void on_control_on_all_clicked();
	void on_control_off_all_clicked();

	//motion control test
	void on_left_motion_button_clicked();
	void on_right_motion_button_clicked();

	// decision module
	void on_decision_process_on_button_clicked();
	void on_decision_process_off_button_clicked();
	void on_update_button_clicked();
	void on_remote_control_on_clicked();
	void on_autonomous_control_on_clicked();
	void on_initialize_flag_sensor_clicked();

	//walking_module
	void on_online_walking_module_clicked();
	void on_none_clicked();

	////command
	void on_turn_left_clicked();
	void on_turn_right_clicked();

	void on_left_clicked();
	void on_right_clicked();

	void on_forward_clicked();
	void on_backward_clicked();

	void on_stop_clicked();

	//parameter
	void on_apply_data_clicked();
	void on_balance_param_apply_clicked();
	void on_joint_feedback_gain_clicked();

	void keyPressEvent( QKeyEvent* event );

	private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	// initialize sensor variables
	std_msgs::Bool ft_init_msg;
	QString temp_check_state;

	// graph variables
	QTimer *dataTimer;
	QFont legendFont;
	double key;

	std_msgs::Int32 pattern_msg; // pattern command msg
	std_msgs::String module_msg; // module on off command msg
	//<------------------------------------------------------------------- offset module -->
	// offset module msg //////////

	std_msgs::Bool save_onoff_msg_;
	std_msgs::Int8 joint_select_msg_;
	std_msgs::Int16MultiArray change_joint_value_msg_;
	std_msgs::Float64MultiArray offset_joint_value_msg_;
	atest_gui::command read_joint_value_msg_; // motor joint value read msg

	// offset_joint variables /////////

	// 독립 제어 용 변수
	QString offset_joint_str_; // 사용자에서 joint 번호를 string 으로 받아오는 변수
	int16_t offset_joint_int16_;   // string -> int16 으로 변환
	QString offset_change_value_str_;
	int16_t offset_change_value_int16_;

	int16_t new_offset_final_value_; // 최종적으로 다이나믹셀에 들어가는 오프셋 명령


	QString offset_DataToRad_str_;// 프레임워크 원점 조정을 위한 0-4095 에서 rad 으로 변환
	int16_t offset_DataToRad_int16_; // 프레임워크 원점 조정을 위한 0-4095 에서 rad 으로 변환 정수형으로 변수

	double offset_joint_value_[30];
	int dynamixel_standard_data[26];// 이론상 영점의 위치

	//<------------------------------------------------------------------------------------->

	//<------------------------------------------------------------------- base module -->
	std_msgs::String init_pose_msg; // init_pose command msg

	//<------------------------------------------------------------------- pose module -->
	std_msgs::Float64MultiArray desired_pose_msg; // desired_pose command msg
	std_msgs::Float64MultiArray desired_pose_waist_msg; // desired_pose command msg
	std_msgs::Float64MultiArray desired_pose_head_msg; // desired_pose command msg
	std_msgs::Float64MultiArray desired_pose_arm_msg; // desired_pose command msg

	atest_gui::command read_p_gain_value_msg_; // motor joint value read msg
	std_msgs::Int16MultiArray gain_adjustment_msg;
	std_msgs::Bool final_gain_save_msg;
	int p_gain_value_[30];
	int id_select;

	//<------------------------------------------------------------------- motion module -->
	// balance on off msg
	diana_msgs::BalanceParam diana_balance_parameter_msg;

	// center_change msg
	diana_msgs::CenterChange center_change_msg;
	double updating_duration;

	double cob_x_offset_m;
	double cob_y_offset_m;

	double foot_roll_gyro_p_gain;
	double foot_roll_gyro_d_gain;
	double foot_pitch_gyro_p_gain;
	double foot_pitch_gyro_d_gain;

	double foot_roll_angle_p_gain;
	double foot_roll_angle_d_gain;
	double foot_pitch_angle_p_gain;
	double foot_pitch_angle_d_gain;

	double foot_x_force_p_gain;
	double foot_x_force_d_gain;

	double foot_y_force_p_gain;
	double foot_y_force_d_gain;

	double foot_z_force_p_gain;
	double foot_z_force_d_gain;

	double foot_roll_torque_p_gain;
	double foot_roll_torque_d_gain;

	double foot_pitch_torque_p_gain;
	double foot_pitch_torque_d_gain;


	double roll_gyro_cut_off_frequency;
	double pitch_gyro_cut_off_frequency;


	double roll_angle_cut_off_frequency;
	double pitch_angle_cut_off_frequency;

	double foot_x_force_cut_off_frequency;
	double foot_y_force_cut_off_frequency;
	double foot_z_force_cut_off_frequency;
	double foot_roll_torque_cut_off_frequency;
	double foot_pitch_torque_cut_off_frequency;


	double foot_copFz_p_gain;
	double foot_copFz_d_gain;

	int direction;
	//<------------------------------------------------------------------- upper module -->
	// upper balance on off msg
	diana_msgs::BalanceParamWaist diana_balance_parameter_waist_msg;

	double updating_duration_waist;

	double waist_roll_gyro_p_gain;
	double waist_roll_gyro_d_gain;
	double waist_yaw_gyro_p_gain;
	double waist_yaw_gyro_d_gain;
	double waist_copFz_p_gain;
	double waist_copFz_d_gain;

	// arm balance on off msg
	diana_msgs::BalanceParamArm diana_balance_parameter_arm_msg;
	double updating_duration_arm;
	double arm_roll_gyro_p_gain;
	double arm_roll_gyro_d_gain;
	double arm_pitch_gyro_p_gain;
	double arm_pitch_gyro_d_gain;
	double arm_yaw_gyro_p_gain;
	double arm_yaw_gyro_d_gain;
	double arm_copFz_p_gain;
	double arm_copFz_d_gain;

	std_msgs::Bool head_balance_msg;

	//<------------------------------------------------------------------- decision module -->
	std_msgs::Bool ready_check_msg;
	std_msgs::Bool update_msg;
	std_msgs::Bool mode_change_msg;
	std_msgs::Bool init_check_msg;

	std_msgs::Bool remote_time_msg;

	QVector<double> r_1, r_2;

	//<------------------------------------------------------------------- walking module -->

	diana_foot_step_generator::FootStepCommand foot_step_command_msg;
	diana_online_walking_module_msgs::SetBalanceParam set_balance_param_msg;
	diana_online_walking_module_msgs::SetJointFeedBackGain joint_feedback_gain_msg;


};

}  // namespace atest_gui

#endif // atest_gui_MAIN_WINDOW_H
