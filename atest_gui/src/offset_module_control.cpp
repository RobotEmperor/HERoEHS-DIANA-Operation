/*
 * offset_module_control.cpp
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

///////////// offset module /////////////////////////////////////////////////////////
void MainWindow::on_read_data_clicked() { // 모든 조인트 값을 읽어온다.
	//<------------------------------------------------->
	qnode.read_joint_value_client.call(read_joint_value_msg_);// 저장된 위치값을 요청하고 받아온다.

	//<-- 각 값들을 GUI에 표시 -->
	ui.lcd_read_joint_1->display(read_joint_value_msg_.response.dxl_state[1]);
	ui.lcd_read_joint_2->display(read_joint_value_msg_.response.dxl_state[2]);
	ui.lcd_read_joint_3->display(read_joint_value_msg_.response.dxl_state[3]);
	ui.lcd_read_joint_4->display(read_joint_value_msg_.response.dxl_state[4]);
	ui.lcd_read_joint_5->display(read_joint_value_msg_.response.dxl_state[5]);

	ui.lcd_read_joint_6->display(read_joint_value_msg_.response.dxl_state[6]);
	ui.lcd_read_joint_7->display(read_joint_value_msg_.response.dxl_state[7]);
	ui.lcd_read_joint_8->display(read_joint_value_msg_.response.dxl_state[8]);
	ui.lcd_read_joint_9->display(read_joint_value_msg_.response.dxl_state[9]);
	ui.lcd_read_joint_10->display(read_joint_value_msg_.response.dxl_state[10]);

	ui.lcd_read_joint_11->display(read_joint_value_msg_.response.dxl_state[11]);
	ui.lcd_read_joint_12->display(read_joint_value_msg_.response.dxl_state[12]);
	ui.lcd_read_joint_13->display(read_joint_value_msg_.response.dxl_state[13]);
	ui.lcd_read_joint_14->display(read_joint_value_msg_.response.dxl_state[14]);
	ui.lcd_read_joint_15->display(read_joint_value_msg_.response.dxl_state[15]);

	ui.lcd_read_joint_16->display(read_joint_value_msg_.response.dxl_state[16]);
	ui.lcd_read_joint_17->display(read_joint_value_msg_.response.dxl_state[17]);
	ui.lcd_read_joint_18->display(read_joint_value_msg_.response.dxl_state[18]);
	ui.lcd_read_joint_19->display(read_joint_value_msg_.response.dxl_state[19]);
	ui.lcd_read_joint_20->display(read_joint_value_msg_.response.dxl_state[20]);

	ui.lcd_read_joint_21->display(read_joint_value_msg_.response.dxl_state[21]);
	ui.lcd_read_joint_22->display(read_joint_value_msg_.response.dxl_state[22]);
	ui.lcd_read_joint_23->display(read_joint_value_msg_.response.dxl_state[23]);
	ui.lcd_read_joint_24->display(read_joint_value_msg_.response.dxl_state[24]);
	ui.lcd_read_joint_25->display(read_joint_value_msg_.response.dxl_state[25]);

	for(int i=1;i<26;i++)
	{
		dynamixel_standard_data[i] = 0;
	}
}

void MainWindow::on_plus_button_clicked()
{
	int data[26] = {0,3320,760,760,3320,760,3320,760,3320,2040,2040,
			3068,1020,3068,1020,2040,2040,3926,170,2720,1360,2040,2040,
			2040,2040,2040};//값 최대

	offset_joint_str_ = ui.offset_id->text();
	offset_joint_int16_ = offset_joint_str_.toInt();

	offset_change_value_str_ = ui.offset_value->text();
	offset_change_value_int16_ = offset_change_value_str_.toInt();

	joint_select_msg_.data = offset_joint_int16_;
	qnode.joint_select_pub.publish(joint_select_msg_);

	qnode.read_joint_value_client.call(read_joint_value_msg_);

	new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_] + offset_change_value_int16_; // 읽어온 현재 위치 값에서 변화된 값을 더해 새로운 값을 얻는다. 예) 1024(읽어온값) + 10 (변화시키고자한 값)

	if(new_offset_final_value_ > data[offset_joint_int16_]) // 4095을 넘어가는 것을 방지
		new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_]; // 지금 값 유지
	else
		new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_]+ offset_change_value_int16_; // 변화 값 저장

	ROS_INFO("%d",new_offset_final_value_ );

	change_joint_value_msg_.data.push_back(new_offset_final_value_); // 메세지에 저장
	qnode.change_joint_value_pub.publish(change_joint_value_msg_); // 메세지 전송

	change_joint_value_msg_.data.clear();


}

void MainWindow::on_minus_button_clicked()
{
	int data[26] = {0,-760,-3320,-3320,-760,-3320,-760,-3320,-760,-2040,-2040,
			-1020,-3060,-1020,-3060,-2040,-2040,-170,-3925,-1360,-2720,-2040,-2040,
			-2040,-2040,-2040};// 0 값 최소

	offset_joint_str_ = ui.offset_id->text();
	offset_joint_int16_ = offset_joint_str_.toInt();

	offset_change_value_str_ = ui.offset_value->text();
	offset_change_value_int16_ = offset_change_value_str_.toInt();

	joint_select_msg_.data = offset_joint_int16_;
	qnode.joint_select_pub.publish(joint_select_msg_);

	qnode.read_joint_value_client.call(read_joint_value_msg_);

	new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_] - offset_change_value_int16_;

	if(new_offset_final_value_ < data[offset_joint_int16_])
	{
		new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_];
		ROS_INFO("END");
	}
	else
		new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_] - offset_change_value_int16_;

	change_joint_value_msg_.data.push_back(new_offset_final_value_); // 메세지에 저장
	qnode.change_joint_value_pub.publish(change_joint_value_msg_);

	change_joint_value_msg_.data.clear();
}
void MainWindow::on_read_data_one_clicked() {

	offset_joint_str_ = ui.offset_id->text();
	offset_joint_int16_ = offset_joint_str_.toInt();

	joint_select_msg_.data = offset_joint_int16_;
	qnode.joint_select_pub.publish(joint_select_msg_);

	qnode.read_joint_value_client.call(read_joint_value_msg_);

	ui.lcd_read_data->display(read_joint_value_msg_.response.dxl_state[offset_joint_int16_]);

}
void MainWindow::on_initial_button_clicked()
{
	double temp_ratio = 0;
	double degTorad = 0.0;
	QString DataToRad_str_temp;

	offset_joint_str_ = ui.offset_id->text();
	offset_joint_int16_ = offset_joint_str_.toInt();

	qnode.read_joint_value_client.call(read_joint_value_msg_);
	offset_DataToRad_int16_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_];

	ui.lcd_read_data->display(read_joint_value_msg_.response.dxl_state[offset_joint_int16_]);
	ui.lcd_read_data_2->display(dynamixel_standard_data[offset_joint_int16_]);

	switch (offset_joint_int16_)
	{
	case 10 : temp_ratio = 0.088/4.0;
	break;
	case 23 : temp_ratio = 0.088/2.5;
	break;
	case 24 : temp_ratio = 0.088;
	break;
	case 25 : temp_ratio = 0.088;
	break;
	default : temp_ratio = 0.088/3.0;
	break;
	}
	if(offset_joint_int16_< 9)
		temp_ratio = 0.088/2.5;

	if(offset_joint_int16_ == 3 || offset_joint_int16_ == 4)
		temp_ratio = 0.088/1.875;

	degTorad  = static_cast<double> (((offset_DataToRad_int16_ - dynamixel_standard_data[offset_joint_int16_])*temp_ratio*M_PI)/180);
	offset_joint_value_[offset_joint_int16_] = degTorad;

	DataToRad_str_temp = QString::number(degTorad);

	ui.lineEdit_offset->setText(DataToRad_str_temp);
}

void MainWindow::on_save_onoff_clicked() {

	save_onoff_msg_.data = true;
	qnode.save_onoff_pub.publish(save_onoff_msg_);
}

void MainWindow::on_send_offset_value_clicked() {

	for(int i=0;i<30;i++)
	{
		offset_joint_value_msg_.data.push_back(offset_joint_value_[i]);
	}

	qnode.offset_joint_value_pub.publish(offset_joint_value_msg_);

	offset_joint_value_msg_.data.clear();
}
}
