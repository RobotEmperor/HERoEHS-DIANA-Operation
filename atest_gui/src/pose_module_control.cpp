/*
 * pose_module_control.cpp
 *
 *  Created on: Dec 20, 2017
 *      Author: robotemperor
 */




#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <stdio.h>
#include "../include/atest_gui/main_window.hpp"


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace atest_gui {
using namespace std;
using namespace Qt;


// POSE Module gui /////////////////////////////////////////////////////////
void MainWindow::on_l_hip_pitch_button_clicked()  //11
{
	id_select = 11;
	ui.joint_name_edit->setText("l_hip_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[11]));
}
void MainWindow::on_l_hip_roll_button_clicked()  //13
{
	id_select = 13;
	ui.joint_name_edit->setText("l_hip_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[13]));
}
void MainWindow::on_l_hip_yaw_button_clicked()  //15
{
	id_select = 15;
	ui.joint_name_edit->setText("l_hip_yaw");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[15]));
}
void MainWindow::on_l_knee_pitch_button_clicked()  //17
{
	id_select = 17;
	ui.joint_name_edit->setText("l_knee_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[17]));
}
void MainWindow::on_l_ankle_pitch_button_clicked()  // 19
{
	id_select = 19;
	ui.joint_name_edit->setText("l_ankle_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[19]));
}
void MainWindow::on_l_ankle_roll_button_clicked()  // 21
{
	id_select = 21;
	ui.joint_name_edit->setText("l_ankle_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[21]));
}
void MainWindow::on_r_hip_pitch_button_clicked()  // 12
{
	id_select = 12;
	ui.joint_name_edit->setText("r_hip_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[12]));
}
void MainWindow::on_r_hip_roll_button_clicked()  // 14
{
	id_select = 14;
	ui.joint_name_edit->setText("r_hip_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[14]));
}
void MainWindow::on_r_hip_yaw_button_clicked()  // 16
{
	id_select = 16;
	ui.joint_name_edit->setText("r_hip_yaw");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[16]));
}
void MainWindow::on_r_knee_pitch_button_clicked()  // 18
{
	id_select = 18;
	ui.joint_name_edit->setText("r_knee_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[18]));
}
void MainWindow::on_r_ankle_pitch_button_clicked()  // 20
{
	id_select = 20;
	ui.joint_name_edit->setText("r_ankle_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[20]));
}
void MainWindow::on_r_ankle_roll_button_clicked()  // 22
{
	id_select = 22;
	ui.joint_name_edit->setText("r_ankle_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[22]));
}
void MainWindow::on_head_yaw_button_clicked()  // 23
{
	id_select = 23;
	ui.joint_name_edit->setText("head_yaw");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[23]));
}
void MainWindow::on_head_pitch_button_clicked()  // 24
{
	id_select = 24;
	ui.joint_name_edit->setText("head_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[24]));
}
void MainWindow::on_head_roll_button_clicked()  // 25
{
	id_select = 25;
	ui.joint_name_edit->setText("head_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[25]));
}
void MainWindow::on_waist_roll_button_clicked()  // 9
{
	id_select = 9;
	ui.joint_name_edit->setText("waist_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[9]));
}
void MainWindow::on_waist_yaw_button_clicked()  // 10
{
	id_select = 10;
	ui.joint_name_edit->setText("waist_yaw");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[10]));
}
void MainWindow::on_l_shoulder_pitch_button_clicked()  // 1
{
	id_select = 1;
	ui.joint_name_edit->setText("l_shoulder_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[1]));
}
void MainWindow::on_l_shoulder_roll_button_clicked()  // 3
{
	id_select = 3;
	ui.joint_name_edit->setText("l_shoulder_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[3]));
}
void MainWindow::on_l_elbow_pitch_button_clicked()  // 5
{
	id_select = 5;
	ui.joint_name_edit->setText("l_elbow_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[5]));
}
void MainWindow::on_r_shoulder_pitch_button_clicked()  // 2
{
	id_select = 2;
	ui.joint_name_edit->setText("r_shoulder_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[2]));
}
void MainWindow::on_r_shoulder_roll_button_clicked()  // 4
{
	id_select = 4;
	ui.joint_name_edit->setText("r_shoulder_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[4]));
}
void MainWindow::on_r_elbow_pitch_button_clicked()  // 6
{
	id_select = 6;
	ui.joint_name_edit->setText("r_elbow_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[6]));
}
void MainWindow::on_initial_p_gain_load_button_clicked()
{
	qnode.read_p_gain_value_client.call(read_p_gain_value_msg_);// 저장된 위치값을 요청하고 받아온다.
	for(int id = 1; id < 30; id++)
	{
		p_gain_value_[id] = read_p_gain_value_msg_.response.dxl_state[id];
	}
}
void MainWindow::on_final_gain_save_button_clicked()
{
	final_gain_save_msg.data = true;

	qnode.final_gain_save_pub.publish(final_gain_save_msg);

}
void MainWindow::on_gain_adjustment_button_clicked()
{
	QString str_p_gain = ui.p_gain_edit->text();
	int data1 = str_p_gain.toInt();

	gain_adjustment_msg.data.push_back(id_select);
	gain_adjustment_msg.data.push_back(data1);
	qnode.gain_adjustment_pub.publish(gain_adjustment_msg);

	gain_adjustment_msg.data.clear();
}


/// POSE MODULE///////////////////////////////////////////////////////////////////////
void MainWindow::on_change_kinematics_1_1_clicked() {
	//  1 pose   11 자 ////////////////////////////////////////////////////
	QString data_str_temp1;
	QString data_str_temp2;
	QString data_str_temp3;

	QString data_str_temp4;
	QString data_str_temp5;
	QString data_str_temp6;

	QString data_str_temp7;
	QString data_str_temp8;
	QString data_str_temp9;

	QString data_str_temp10;
	QString data_str_temp11;
	QString data_str_temp12;

	QString str_waist_roll_temp;
	QString str_waist_yaw_temp;

	QString str_head_yaw_temp;
	QString str_head_pitch_temp;
	QString str_head_roll_temp;

	QString str_l_arm_x_temp;
	QString str_l_arm_y_temp;
	QString str_l_arm_z_temp;

	QString str_r_arm_x_temp;
	QString str_r_arm_y_temp;
	QString str_r_arm_z_temp;
	// arm ///////////////////////////////////////////
	double l_arm_x = 0.17677;
	double l_arm_y = 0.14;
	double l_arm_z = -0.35;

	double r_arm_x = 0.17677;
	double r_arm_y = -0.14;
	double r_arm_z = -0.35;
	// head //////////////////////////////////////////
	double dHead_yaw = 0;
	dHead_yaw  = (dHead_yaw*M_PI)/180;

	double dHead_pitch = -10;
	dHead_pitch  = (dHead_pitch*M_PI)/180;

	double dHead_roll = 0;
	dHead_roll  = (dHead_roll*M_PI)/180;

	// waist ///
	double dWaist_roll = 0;
	dWaist_roll = (dWaist_roll*M_PI)/180;

	double dWaist_yaw = 0;
	dWaist_yaw = (dWaist_yaw*M_PI)/180;

	/// leg ////
	double data1 = 0;
	double data2 = 0.105;
	double data3 = -0.52;
	double data4 = 0;
	data4 = (data4*M_PI)/180;

	double data5 = 0;
	data5 = (data5*M_PI)/180;

	double data6 = 0;
	data6 = (data6*M_PI)/180;

	double data7 = 0;
	double data8 = -0.105;
	double data9 = -0.52;
	double data10 = 0;
	data10 = (data10*M_PI)/180;

	double data11 = 0;
	data11 = (data11*M_PI)/180;
	double data12 = 0;
	data12 = (data12*M_PI)/180;
	data_str_temp1 = QString::number(data1);
	data_str_temp2 = QString::number(data2);
	data_str_temp3 = QString::number(data3);

	data_str_temp4 = QString::number((data4*180)/M_PI);
	data_str_temp5 = QString::number((data5*180)/M_PI);
	data_str_temp6 = QString::number((data6)*180/M_PI);

	data_str_temp7 = QString::number(data7);
	data_str_temp8 = QString::number(data8);
	data_str_temp9 = QString::number(data9);

	data_str_temp10 = QString::number((data10)*180/M_PI);
	data_str_temp11 = QString::number((data11)*180/M_PI);
	data_str_temp12 = QString::number((data12)*180/M_PI);
	// waist ///////////////////////////////////////////
	str_waist_roll_temp = QString::number((dWaist_roll)*180/M_PI);
	str_waist_yaw_temp  = QString::number((dWaist_yaw)*180/M_PI);

	//head ////////////////////////////////////////////
	str_head_yaw_temp   = QString::number((dHead_yaw)*180/M_PI);
	str_head_pitch_temp = QString::number((dHead_pitch)*180/M_PI);
	str_head_roll_temp  = QString::number((dHead_roll)*180/M_PI);

	// arm ////////////////////////////////////////////
	str_l_arm_x_temp = QString::number(l_arm_x);
	str_l_arm_y_temp = QString::number(l_arm_y);
	str_l_arm_z_temp = QString::number(l_arm_z);
	str_r_arm_x_temp = QString::number(r_arm_x);
	str_r_arm_y_temp = QString::number(r_arm_y);
	str_r_arm_z_temp = QString::number(r_arm_z);

	// display ///
	ui.Edit_X->setText(data_str_temp1);
	ui.Edit_Y->setText(data_str_temp2);
	ui.Edit_Z->setText(data_str_temp3);

	ui.Edit_Z_angle->setText(data_str_temp4);
	ui.Edit_Y_angle->setText(data_str_temp5);
	ui.Edit_X_angle->setText(data_str_temp6);

	ui.Edit_X_2->setText(data_str_temp7);
	ui.Edit_Y_2->setText(data_str_temp8);
	ui.Edit_Z_2->setText(data_str_temp9);

	ui.Edit_Z_angle_2->setText(data_str_temp10);
	ui.Edit_Y_angle_2->setText(data_str_temp11);
	ui.Edit_X_angle_2->setText(data_str_temp12);

	ui.Edit_waist_9 ->setText(str_waist_yaw_temp);
	ui.Edit_waist_10->setText(str_waist_roll_temp);

	ui.Edit_head_23->setText(str_head_yaw_temp);
	ui.Edit_head_24->setText(str_head_pitch_temp);
	ui.Edit_head_25->setText(str_head_roll_temp);

	ui.Edit_left_x ->setText(str_l_arm_x_temp);
	ui.Edit_left_y ->setText(str_l_arm_y_temp);
	ui.Edit_left_z ->setText(str_l_arm_z_temp);
	ui.Edit_right_x->setText(str_r_arm_x_temp);
	ui.Edit_right_y->setText(str_r_arm_y_temp);
	ui.Edit_right_z->setText(str_r_arm_z_temp);

	desired_pose_msg.data.push_back(data1);
	desired_pose_msg.data.push_back(data2);
	desired_pose_msg.data.push_back(data3);
	desired_pose_msg.data.push_back(data4);
	desired_pose_msg.data.push_back(data5);
	desired_pose_msg.data.push_back(data6);

	desired_pose_msg.data.push_back(data7);
	desired_pose_msg.data.push_back(data8);
	desired_pose_msg.data.push_back(data9);
	desired_pose_msg.data.push_back(data10);
	desired_pose_msg.data.push_back(data11);
	desired_pose_msg.data.push_back(data12);
	desired_pose_msg.data.push_back(4);

	desired_pose_waist_msg.data.push_back(dWaist_yaw);
	desired_pose_waist_msg.data.push_back(dWaist_roll);
	desired_pose_waist_msg.data.push_back(4);
	desired_pose_waist_msg.data.push_back(4);


	desired_pose_head_msg.data.push_back(dHead_yaw);
	desired_pose_head_msg.data.push_back(dHead_pitch);
	desired_pose_head_msg.data.push_back(dHead_roll);
	desired_pose_head_msg.data.push_back(4);

	desired_pose_arm_msg.data.push_back(l_arm_x);
	desired_pose_arm_msg.data.push_back(l_arm_y);
	desired_pose_arm_msg.data.push_back(l_arm_z);
	desired_pose_arm_msg.data.push_back(r_arm_x);
	desired_pose_arm_msg.data.push_back(r_arm_y);
	desired_pose_arm_msg.data.push_back(r_arm_z);
	desired_pose_arm_msg.data.push_back(4);

	qnode.desired_pose_arm_pub.publish(desired_pose_arm_msg);
	desired_pose_arm_msg.data.clear();

	qnode.desired_pose_head_pub.publish(desired_pose_head_msg);
	desired_pose_head_msg.data.clear();

	qnode.desired_pose_waist_pub.publish(desired_pose_waist_msg);
	desired_pose_waist_msg.data.clear();

	qnode.desired_pose_pub.publish(desired_pose_msg);
	desired_pose_msg.data.clear();
}
void MainWindow::on_initialize_module_2_clicked() {
	// 2 pose ///////////////////////////////////
	QString data_str_temp1;
	QString data_str_temp2;
	QString data_str_temp3;

	QString data_str_temp4;
	QString data_str_temp5;
	QString data_str_temp6;

	QString data_str_temp7;
	QString data_str_temp8;
	QString data_str_temp9;

	QString data_str_temp10;
	QString data_str_temp11;
	QString data_str_temp12;

	QString str_waist_roll_temp;
	QString str_waist_yaw_temp;

	QString str_head_yaw_temp;
	QString str_head_pitch_temp;
	QString str_head_roll_temp;

	QString str_l_arm_x_temp;
	QString str_l_arm_y_temp;
	QString str_l_arm_z_temp;

	QString str_r_arm_x_temp;
	QString str_r_arm_y_temp;
	QString str_r_arm_z_temp;
	// arm ///////////////////////////////////////////
	double l_arm_x = 0.17677;
	double l_arm_y = 0.14;
	double l_arm_z = -0.35;

	double r_arm_x = 0.17677;
	double r_arm_y = -0.14;
	double r_arm_z = -0.35;
	// head //////////////////////////////////////////
	double dHead_yaw = 0;
	dHead_yaw  = (dHead_yaw*M_PI)/180;

	double dHead_pitch = -10;
	dHead_pitch  = (dHead_pitch*M_PI)/180;

	double dHead_roll = 0;
	dHead_roll  = (dHead_roll*M_PI)/180;

	// waist /////////////////////////////////////////
	double dWaist_roll = 0;
	dWaist_roll = (dWaist_roll*M_PI)/180;

	double dWaist_yaw = 0;
	dWaist_yaw = (dWaist_yaw*M_PI)/180;

	// leg ///////////////////////////////////////////
	double data1 = 0.1;
	double data2 = 0.255;
	double data3 = -0.51;
	double data4 = -15;
	data4 = (data4*M_PI)/180;

	double data5 = -10;
	data5 = (data5*M_PI)/180;

	double data6 = 15;
	data6 = (data6*M_PI)/180;

	double data7 = 0.1;
	double data8 = -0.255;
	double data9 = -0.51;
	double data10 = 15;
	data10 = (data10*M_PI)/180;

	double data11 = -10;
	data11 = (data11*M_PI)/180;

	double data12 = -15;
	data12 = (data12*M_PI)/180;

	data_str_temp1 = QString::number(data1);
	data_str_temp2 = QString::number(data2);
	data_str_temp3 = QString::number(data3);

	data_str_temp4 = QString::number((data4*180)/M_PI);
	data_str_temp5 = QString::number((data5*180)/M_PI);
	data_str_temp6 = QString::number((data6)*180/M_PI);

	data_str_temp7 = QString::number(data7);
	data_str_temp8 = QString::number(data8);
	data_str_temp9 = QString::number(data9);

	data_str_temp10 = QString::number((data10)*180/M_PI);
	data_str_temp11 = QString::number((data11)*180/M_PI);
	data_str_temp12 = QString::number((data12)*180/M_PI);
	// waist ///////////////////////////////////////////
	str_waist_roll_temp = QString::number((dWaist_roll)*180/M_PI);
	str_waist_yaw_temp  = QString::number((dWaist_yaw)*180/M_PI);

	//head ////////////////////////////////////////////
	str_head_yaw_temp   = QString::number((dHead_yaw)*180/M_PI);
	str_head_pitch_temp = QString::number((dHead_pitch)*180/M_PI);
	str_head_roll_temp  = QString::number((dHead_roll)*180/M_PI);
	// arm /////////////////////////////////////////////
	str_l_arm_x_temp = QString::number(l_arm_x);
	str_l_arm_y_temp = QString::number(l_arm_y);
	str_l_arm_z_temp = QString::number(l_arm_z);
	str_r_arm_x_temp = QString::number(r_arm_x);
	str_r_arm_y_temp = QString::number(r_arm_y);
	str_r_arm_z_temp = QString::number(r_arm_z);
	/////display /////////////////////////////////////
	ui.Edit_X->setText(data_str_temp1);
	ui.Edit_Y->setText(data_str_temp2);
	ui.Edit_Z->setText(data_str_temp3);

	ui.Edit_Z_angle->setText(data_str_temp4);
	ui.Edit_Y_angle->setText(data_str_temp5);
	ui.Edit_X_angle->setText(data_str_temp6);

	ui.Edit_X_2->setText(data_str_temp7);
	ui.Edit_Y_2->setText(data_str_temp8);
	ui.Edit_Z_2->setText(data_str_temp9);

	ui.Edit_Z_angle_2->setText(data_str_temp10);
	ui.Edit_Y_angle_2->setText(data_str_temp11);
	ui.Edit_X_angle_2->setText(data_str_temp12);

	ui.Edit_waist_9->setText(str_waist_yaw_temp);
	ui.Edit_waist_10->setText(str_waist_roll_temp);

	ui.Edit_head_23->setText(str_head_yaw_temp);
	ui.Edit_head_24->setText(str_head_pitch_temp);
	ui.Edit_head_25->setText(str_head_roll_temp);

	ui.Edit_left_x ->setText(str_l_arm_x_temp);
	ui.Edit_left_y ->setText(str_l_arm_y_temp);
	ui.Edit_left_z ->setText(str_l_arm_z_temp);
	ui.Edit_right_x->setText(str_r_arm_x_temp);
	ui.Edit_right_y->setText(str_r_arm_y_temp);
	ui.Edit_right_z->setText(str_r_arm_z_temp);

	desired_pose_msg.data.push_back(data1);
	desired_pose_msg.data.push_back(data2);
	desired_pose_msg.data.push_back(data3);
	desired_pose_msg.data.push_back(data4);
	desired_pose_msg.data.push_back(data5);
	desired_pose_msg.data.push_back(data6);

	desired_pose_msg.data.push_back(data7);
	desired_pose_msg.data.push_back(data8);
	desired_pose_msg.data.push_back(data9);
	desired_pose_msg.data.push_back(data10);
	desired_pose_msg.data.push_back(data11);
	desired_pose_msg.data.push_back(data12);
	desired_pose_msg.data.push_back(4);

	desired_pose_waist_msg.data.push_back(dWaist_yaw);
	desired_pose_waist_msg.data.push_back(dWaist_roll);
	desired_pose_waist_msg.data.push_back(4);
	desired_pose_waist_msg.data.push_back(4);

	desired_pose_head_msg.data.push_back(dHead_yaw );
	desired_pose_head_msg.data.push_back(dHead_pitch);
	desired_pose_head_msg.data.push_back(dHead_roll);
	desired_pose_head_msg.data.push_back(4);

	desired_pose_arm_msg.data.push_back(l_arm_x);
	desired_pose_arm_msg.data.push_back(l_arm_y);
	desired_pose_arm_msg.data.push_back(l_arm_z);
	desired_pose_arm_msg.data.push_back(r_arm_x);
	desired_pose_arm_msg.data.push_back(r_arm_y);
	desired_pose_arm_msg.data.push_back(r_arm_z);
	desired_pose_arm_msg.data.push_back(4);

	qnode.desired_pose_arm_pub.publish(desired_pose_arm_msg);
	desired_pose_arm_msg.data.clear();

	qnode.desired_pose_head_pub.publish(desired_pose_head_msg);
	desired_pose_head_msg.data.clear();

	qnode.desired_pose_waist_pub.publish(desired_pose_waist_msg);
	desired_pose_waist_msg.data.clear();

	qnode.desired_pose_pub.publish(desired_pose_msg);
	desired_pose_msg.data.clear();
}
void MainWindow::on_change_kinematics_2_clicked() // leg change
{
	QString str1 = ui.Edit_X->text();
	double data1 = str1.toDouble();

	QString str2 = ui.Edit_Y->text();
	double data2 = str2.toDouble();

	QString str3 = ui.Edit_Z->text();
	double data3 = str3.toDouble();

	QString str4 = ui.Edit_Z_angle->text();
	double data4 = str4.toDouble();

	data4 = (data4*M_PI)/180;

	QString str5 = ui.Edit_Y_angle->text();
	double data5 = str5.toDouble();

	data5 = (data5*M_PI)/180;

	QString str6 = ui.Edit_X_angle->text();
	double data6 = str6.toDouble();

	data6 = (data6*M_PI)/180;

	QString str7 = ui.Edit_X_2->text();
	double data7 = str7.toDouble();

	QString str8 = ui.Edit_Y_2->text();
	double data8 = str8.toDouble();

	QString str9 = ui.Edit_Z_2->text();
	double data9 = str9.toDouble();

	QString str10 = ui.Edit_Z_angle_2->text();
	double data10 = str10.toDouble();

	data10 = (data10*M_PI)/180;

	QString str11 = ui.Edit_Y_angle_2->text();
	double data11 = str11.toDouble();

	data11 = (data11*M_PI)/180;

	QString str12 = ui.Edit_X_angle_2->text();
	double data12 = str12.toDouble();

	data12 = (data12*M_PI)/180;

	desired_pose_msg.data.push_back(data1);
	desired_pose_msg.data.push_back(data2);
	desired_pose_msg.data.push_back(data3);
	desired_pose_msg.data.push_back(data4);
	desired_pose_msg.data.push_back(data5);
	desired_pose_msg.data.push_back(data6);
	desired_pose_msg.data.push_back(data7);
	desired_pose_msg.data.push_back(data8);
	desired_pose_msg.data.push_back(data9);
	desired_pose_msg.data.push_back(data10);
	desired_pose_msg.data.push_back(data11);
	desired_pose_msg.data.push_back(data12);
	desired_pose_msg.data.push_back(4);

	qnode.desired_pose_pub.publish(desired_pose_msg);

	desired_pose_msg.data.clear();
}
void MainWindow::on_waist_change_button_clicked()
{
	QString str_waist_roll = ui.Edit_waist_10->text();
	double dWaist_roll = str_waist_roll.toDouble();

	dWaist_roll = (dWaist_roll*M_PI)/180;

	QString str_waist_yaw = ui.Edit_waist_9->text();
	double dWaist_yaw = str_waist_yaw.toDouble();

	dWaist_yaw = (dWaist_yaw*M_PI)/180;

	desired_pose_waist_msg.data.push_back(dWaist_yaw);
	desired_pose_waist_msg.data.push_back(dWaist_roll);
	desired_pose_waist_msg.data.push_back(4);
	desired_pose_waist_msg.data.push_back(4);

	qnode.desired_pose_waist_pub.publish(desired_pose_waist_msg);

	desired_pose_waist_msg.data.clear();
}
void MainWindow::on_head_change_button_clicked()
{
	QString str_head_yaw = ui.Edit_head_23->text();
	double dHead_yaw = str_head_yaw.toDouble();
	dHead_yaw  = (dHead_yaw*M_PI)/180;

	QString str_head_pitch = ui.Edit_head_24->text();
	double dHead_pitch = str_head_pitch.toDouble();
	dHead_pitch  = (dHead_pitch*M_PI)/180;

	QString str_head_roll = ui.Edit_head_25->text();
	double dHead_roll = str_head_roll.toDouble();
	dHead_roll  = (dHead_roll*M_PI)/180;

	desired_pose_head_msg.data.push_back(dHead_yaw);
	desired_pose_head_msg.data.push_back(dHead_pitch);
	desired_pose_head_msg.data.push_back(dHead_roll);
	desired_pose_head_msg.data.push_back(4);

	qnode.desired_pose_head_pub.publish(desired_pose_head_msg);

	desired_pose_head_msg.data.clear();
}
void MainWindow::on_arm_change_end_effector_button_clicked()
{
	QString str_l_arm_x = ui.Edit_left_x->text();
	double dl_arm_x = str_l_arm_x.toDouble();


	QString str_l_arm_y = ui.Edit_left_y->text();
	double dl_arm_y = str_l_arm_y.toDouble();


	QString str_l_arm_z = ui.Edit_left_z->text();
	double dl_arm_z = str_l_arm_z.toDouble();


	QString str_r_arm_x = ui.Edit_right_x->text();
	double dr_arm_x = str_r_arm_x.toDouble();


	QString str_r_arm_y = ui.Edit_right_y->text();
	double dr_arm_y = str_r_arm_y.toDouble();


	QString str_r_arm_z = ui.Edit_right_z->text();
	double dr_arm_z = str_r_arm_z.toDouble();


	desired_pose_arm_msg.data.push_back(dl_arm_x);
	desired_pose_arm_msg.data.push_back(dl_arm_y);
	desired_pose_arm_msg.data.push_back(dl_arm_z);

	desired_pose_arm_msg.data.push_back(dr_arm_x);
	desired_pose_arm_msg.data.push_back(dr_arm_y);
	desired_pose_arm_msg.data.push_back(dr_arm_z);

	desired_pose_arm_msg.data.push_back(4);

	qnode.desired_pose_arm_pub.publish(desired_pose_arm_msg);

	desired_pose_arm_msg.data.clear();
}

void MainWindow::on_arm_change_joint_button_clicked()
{
	QString str_l_arm_x_temp;
	QString str_l_arm_y_temp;
	QString str_l_arm_z_temp;

	QString str_r_arm_x_temp;
	QString str_r_arm_y_temp;
	QString str_r_arm_z_temp;
	// arm ///////////////////////////////////////////
	double l_arm_x = 0.17677;
	double l_arm_y = 0.14;
	double l_arm_z = -0.35;

	double r_arm_x = 0.17677;
	double r_arm_y = -0.14;
	double r_arm_z = -0.35;
	// arm /////////////////////////////////////////////
	str_l_arm_x_temp = QString::number(l_arm_x);
	str_l_arm_y_temp = QString::number(l_arm_y);
	str_l_arm_z_temp = QString::number(l_arm_z);
	str_r_arm_x_temp = QString::number(r_arm_x);
	str_r_arm_y_temp = QString::number(r_arm_y);
	str_r_arm_z_temp = QString::number(r_arm_z);
	/////display /////////////////////////////////////
	ui.Edit_left_x ->setText(str_l_arm_x_temp);
	ui.Edit_left_y ->setText(str_l_arm_y_temp);
	ui.Edit_left_z ->setText(str_l_arm_z_temp);
	ui.Edit_right_x->setText(str_r_arm_x_temp);
	ui.Edit_right_y->setText(str_r_arm_y_temp);
	ui.Edit_right_z->setText(str_r_arm_z_temp);

	desired_pose_arm_msg.data.push_back(l_arm_x);
	desired_pose_arm_msg.data.push_back(l_arm_y);
	desired_pose_arm_msg.data.push_back(l_arm_z);
	desired_pose_arm_msg.data.push_back(r_arm_x);
	desired_pose_arm_msg.data.push_back(r_arm_y);
	desired_pose_arm_msg.data.push_back(r_arm_z);
	desired_pose_arm_msg.data.push_back(4);

	qnode.desired_pose_arm_pub.publish(desired_pose_arm_msg);
	desired_pose_arm_msg.data.clear();
}
}
