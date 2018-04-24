/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

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

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent)
, qnode(argc,argv)
{
	qnode.init();
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	legendFont = font();  // start out with MainWindow's font..
	graph_draw(ui.sensor_value_plot_angular, "Angular velocity", "Rad/s", -3, 3 , 10);
	graph_draw(ui.sensor_value_plot_angular_vel_robot, "Angular velocity robot", "Rad/s", -3, 3, 10);
	graph_draw(ui.sensor_value_plot_fl, "Force Sensor Left leg", "N", -3, 3, 10);
	graph_draw(ui.sensor_value_plot_fr2, "Force Sensor Right leg", "N", -3, 3, 10);
	graph_draw(ui.sensor_value_plot_tl, "Torque Sensor Left leg", "Nm", -3, 3, 10);
	graph_draw(ui.sensor_value_plot_tr, "Torque Sensor Right leg", "Nm", -3, 3, 10);

	graph_draw(ui.left_compensation_rpy, "Left compensation_value", "Rad", -1, 1, 10);
	graph_draw(ui.right_compensation_rpy, "Right compensation_value", "Rad", -1, 1, 10);

	graph_draw_none_line(ui.cop_fz_graph, "         COP FZ", "m", -0.5, 0.5, -0.5, 0.5, 10);
	graph_draw_none_line_flag(ui.flag_graph, "      Flag Position with respect to robot origin", "m", -15, 15,-5,15,10);
	graph_draw_top_view(ui.robot_path_graph, "        Top view", "m", -15, 15,0,80,10);

	dataTimer = new QTimer(this);
	connect(dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
	dataTimer->start(0.006); // Interval 0 means to refresh as fast as possible

	connect(ui.slider_center_change, SIGNAL(valueChanged(int)), this, SLOT(center_change(int)));
	connect(ui.slider_edge_change, SIGNAL(valueChanged(int)), this, SLOT(edge_change(int)));

	for(int i=0;i<30;i++)
	{
		offset_joint_value_[i] = 0;
	}
	for(int i=0;i<26;i++)
	{
		dynamixel_standard_data[i] = 0;
	}

	// gain value //
	updating_duration = 0.0;

	cob_x_offset_m = 0.0;
	cob_y_offset_m = 0.0;
	foot_roll_gyro_p_gain = 0.0;
	foot_roll_gyro_d_gain = 0.0;
	foot_pitch_gyro_p_gain = 0.0;
	foot_pitch_gyro_d_gain = 0.0;

	direction = 0;
	r_1.append(0);
	r_2.append(0);
}

MainWindow::~MainWindow()
{

}
/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/
void MainWindow::on_initialize_flag_sensor_clicked()
{
	init_check_msg.data = 1;
	qnode.init_check_pub.publish(init_check_msg);

}
void MainWindow::on_update_button_clicked()
{
	update_msg.data = 1;
	qnode.update_pub.publish(update_msg);

}
void MainWindow::on_remote_control_on_clicked()
{
	mode_change_msg.data = 0;
	qnode.mode_change_pub.publish(mode_change_msg);
}
void MainWindow::on_autonomous_control_on_clicked()
{
	mode_change_msg.data = 1;
	qnode.mode_change_pub.publish(mode_change_msg);
}
void MainWindow::keyPressEvent( QKeyEvent* event )
{
	QString time_change_value_str;
	QString time_change_edge_value_str;
	QString time_change_waist_value_str;
	QString motion_center_value_str;
	QString time_change_center_str;

	double  time_change_value;
	double  time_change_edge_value;
	double  time_change_waist_value;
	double  motion_center_value;
	double time_change_center_value;

	time_change_value_str = ui.time_change->text();
	time_change_value = time_change_value_str.toDouble();

	time_change_edge_value_str = ui.time_change_edge_2->text();
	time_change_edge_value = time_change_edge_value_str.toDouble();

	time_change_waist_value_str = ui.time_change_waist->text();
	time_change_waist_value = time_change_waist_value_str.toDouble();

	motion_center_value_str = ui.motion_center_value_text->text();
	motion_center_value = motion_center_value_str.toDouble();

	time_change_center_str = ui.time_change_center->text();
	time_change_center_value = time_change_center_str.toDouble();


	switch ( event->key() ) {
	case Qt::Key_A:
	{
		center_change_msg.turn_type = "pflug_bogen";
		center_change_msg.center_change = -1;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_D:
	{
		center_change_msg.turn_type = "pflug_bogen";
		center_change_msg.center_change = 1;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_S:
	{
		center_change_msg.turn_type = "carving_turn";
		center_change_msg.center_change = 5;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_Z:
	{
		center_change_msg.turn_type = "carving_turn";
		center_change_msg.center_change = 1;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_X:
	{
		center_change_msg.turn_type = "carving_turn";
		center_change_msg.center_change = 0;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_C:
	{
		center_change_msg.turn_type = "carving_turn";
		center_change_msg.center_change = -1;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_V:
	{
		center_change_msg.turn_type = "carving_turn";
		center_change_msg.center_change = 2;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_2:
	{
		center_change_msg.turn_type = "carving_turn";
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_Q:
	{
		remote_time_msg.data = 1;
		qnode.remote_time_pub.publish(remote_time_msg);
	}
	break;
	case Qt::Key_W:
	{
		remote_time_msg.data = 0;
		qnode.remote_time_pub.publish(remote_time_msg);
	}
	break;
	/*case Qt::Key_6:
	{
		center_change_msg.turn_type = "pflug_bogen";
		center_change_msg.change_type = "center_change";
		center_change_msg.time_change_waist = time_change_waist_value;
		center_change_msg.waist_change = direction*0.2;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_7:
	{
		center_change_msg.turn_type = "pflug_bogen";
		center_change_msg.change_type = "center_change";
		center_change_msg.time_change_waist = time_change_waist_value;
		center_change_msg.waist_change = direction*0.4;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_8:
	{
		center_change_msg.turn_type = "pflug_bogen";
		center_change_msg.change_type = "center_change";
		center_change_msg.time_change_waist = time_change_waist_value;
		center_change_msg.waist_change = direction*0.6;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_9:
	{
		center_change_msg.turn_type = "pflug_bogen";
		center_change_msg.change_type = "center_change";
		center_change_msg.time_change_waist = time_change_waist_value;
		center_change_msg.waist_change = direction*0.8;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;
	case Qt::Key_0:
	{
		center_change_msg.turn_type = "pflug_bogen";
		center_change_msg.change_type = "center_change";
		center_change_msg.time_change_waist = time_change_waist_value;
		center_change_msg.waist_change = direction*1;
		qnode.center_change_pub.publish(center_change_msg);
	}
	break;*/


	default:
		event->ignore();
		break;
	}
}
void MainWindow::realtimeDataSlot()
{
	static QTime time(QTime::currentTime());
	// calculate two new data points:
	key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
	static double lastPointKey = 0;

	if (key-lastPointKey > 0.006) //
	{
		graph_draw_update(ui.sensor_value_plot_angular_vel_robot, qnode.tf_gyro_value_x, qnode.tf_gyro_value_y, qnode.tf_gyro_value_z);
		graph_draw_update(ui.sensor_value_plot_angular, qnode.currentGyroX_gui, qnode.currentGyroY_gui, qnode.currentGyroZ_gui);
		graph_draw_update(ui.sensor_value_plot_fl, qnode.currentForceX_l_gui, qnode.currentForceY_l_gui, qnode.currentForceZ_l_gui);
		graph_draw_update(ui.sensor_value_plot_fr2, qnode.currentForceX_r_gui, qnode.currentForceY_r_gui, qnode.currentForceZ_r_gui);
		graph_draw_update(ui.sensor_value_plot_tl, qnode.currentTorqueX_l_gui, qnode.currentTorqueY_l_gui, qnode.currentTorqueZ_l_gui);
		graph_draw_update(ui.sensor_value_plot_tr, qnode.currentTorqueX_r_gui, qnode.currentTorqueY_r_gui, qnode.currentTorqueZ_r_gui);
		graph_draw_update(ui.left_compensation_rpy, qnode.l_compensation_roll, qnode.l_compensation_pitch, qnode.l_compensation_yaw);// must be modified
		graph_draw_update(ui.right_compensation_rpy, qnode.r_compensation_roll, qnode.r_compensation_pitch, qnode.r_compensation_yaw);//

		graph_draw_update_none_line(ui.cop_fz_graph, qnode.current_cop_fz_x, qnode.current_cop_fz_y, qnode.reference_cop_fz_x,qnode.reference_cop_fz_y,
				qnode.lf_point_x, qnode.lf_point_y, qnode.rf_point_x, qnode.rf_point_y);
		graph_draw_update_none_line_flag(ui.flag_graph, 0, 0, qnode.current_flag_position1[0], qnode.current_flag_position1[1], qnode.current_flag_position2[0], qnode.current_flag_position2[1]);
		graph_draw_update_top_view(ui.robot_path_graph, qnode.top_view_robot_x, qnode.top_view_robot_y, qnode.in_flag_top_view_x, qnode.in_flag_top_view_y , qnode.out_flag_top_view_x, qnode.out_flag_top_view_y);

		lastPointKey = key;
	}
	// make key axis range scroll with the data (at a constant range size of 8):
	graph_draw_clean(ui.sensor_value_plot_angular);
	graph_draw_clean(ui.sensor_value_plot_angular_vel_robot);
	graph_draw_clean(ui.sensor_value_plot_fl);
	graph_draw_clean(ui.sensor_value_plot_fr2);
	graph_draw_clean(ui.sensor_value_plot_tl);
	graph_draw_clean(ui.sensor_value_plot_tr);
	graph_draw_clean(ui.left_compensation_rpy);
	graph_draw_clean(ui.right_compensation_rpy);

	//graph_draw_clean_none_line(ui.cop_fz_graph);

	// xyz 실시간 좌표 표시
	ui.lcd_left_x ->display(qnode.l_arm_current_state_x);
	ui.lcd_left_y ->display(qnode.l_arm_current_state_y);
	ui.lcd_left_z ->display(qnode.l_arm_current_state_z);
	ui.lcd_right_x->display(qnode.r_arm_current_state_x);
	ui.lcd_right_y->display(qnode.r_arm_current_state_y);
	ui.lcd_right_z->display(qnode.r_arm_current_state_z);

	// cop value display
	ui.lcd_current_cop_fz_x  ->display(qnode.current_cop_fz_x);
	ui.lcd_current_cop_fz_y  ->display(qnode.current_cop_fz_y);
	ui.lcd_reference_cop_fz_x->display(qnode.reference_cop_fz_x);
	ui.lcd_reference_cop_fz_y->display(qnode.reference_cop_fz_y);

	// flag position display
	ui.lcd_flag1_x -> display(qnode.current_flag_position1[0]);
	ui.lcd_flag1_y -> display(qnode.current_flag_position1[1]);

	ui.lcd_flag2_x -> display(qnode.current_flag_position2[0]);
	ui.lcd_flag2_y -> display(qnode.current_flag_position2[1]);

	ui.lcd_robot_x -> display(qnode.top_view_robot_x);
	ui.lcd_robot_y -> display(qnode.top_view_robot_y);
	/*
	ui.lcd_flag2_x -> display(qnode.current_flag_position.data[1].position.x);
	ui.lcd_flag2_y -> display(qnode.current_flag_position.data[1].position.y);

	ui.lcd_flag3_x -> display(qnode.current_flag_position.data[2].position.x);
	ui.lcd_flag3_y -> display(qnode.current_flag_position.data[2].position.y);
	ui.lcd_flag4_x -> display(qnode.current_flag_position.data[3].position.x);
	ui.lcd_flag4_y -> display(qnode.current_flag_position.data[3].position.y);
	 */

	if(qnode.ft_init_done_check)
	{
		temp_check_state = "FT Sensor Initialize Completed";
		ui.sensor_state->setText(temp_check_state);
	}
	else
	{
		temp_check_state = "Need to initialize";
		ui.sensor_state->setText(temp_check_state);
	}
}
void MainWindow::graph_draw(QCustomPlot *ui_graph, const QString title, const QString unit, int min_value, int max_value, int tick_count)
{
	ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
	ui_graph->legend->setVisible(true);
	legendFont.setPointSize(9); // and make a bit smaller for legend
	ui_graph->legend->setFont(legendFont);
	ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));
	// by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
	ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

	ui_graph->plotLayout()->insertRow(0);
	ui_graph->plotLayout()->addElement(0, 0, new QCPTextElement(ui_graph, title, QFont("sans", 12, QFont::Bold)));

	ui_graph->addGraph();
	ui_graph->graph(0)->setPen(QPen(QColor(40, 110, 255)));
	ui_graph->graph(0)->setName("X");
	ui_graph->addGraph();
	ui_graph->graph(1)->setPen(QPen(QColor(255, 0, 0)));
	ui_graph->graph(1)->setName("Y");
	ui_graph->addGraph();
	ui_graph->graph(2)->setPen(QPen(QColor(0, 0, 0)));
	ui_graph->graph(2)->setName("Z");
	ui_graph->xAxis->setLabel("Time(s)");
	ui_graph->yAxis->setLabel(unit);

	QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
	timeTicker->setTimeFormat("%s");
	timeTicker->setFieldWidth(timeTicker->tuSeconds,1);
	timeTicker->setTickCount(tick_count);
	ui_graph->xAxis->setTicker(timeTicker);

	ui_graph->axisRect()->setupFullAxesBox();
	ui_graph->yAxis->setRange(min_value, max_value);
}
void MainWindow::graph_draw_update(QCustomPlot *ui_graph, double valueX, double valueY, double valueZ)
{
	// add data to lines:
	ui_graph->graph(0)->addData(key, valueX);
	ui_graph->graph(1)->addData(key, valueY);
	ui_graph->graph(2)->addData(key, valueZ);

	ui_graph->graph(0)->rescaleValueAxis(true);
	ui_graph->graph(1)->rescaleValueAxis(true);
	ui_graph->graph(2)->rescaleValueAxis(true);
}
void MainWindow::graph_draw_clean(QCustomPlot *ui_graph)
{
	ui_graph->xAxis->setRange(key, 8, Qt::AlignRight);
	ui_graph->replot();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::graph_draw_none_line(QCustomPlot *ui_graph, const QString title, const QString unit, double min_value_x, double max_value_x, double min_value_y, double max_value_y, int tick_count)
{
	ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
	ui_graph->legend->setVisible(false);
	legendFont.setPointSize(9); // and make a bit smaller for legend
	ui_graph->legend->setFont(legendFont);
	ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));
	// by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
	ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignRight);

	ui_graph->plotLayout()->insertRow(0);
	ui_graph->plotLayout()->addElement(0, 0, new QCPTextElement(ui_graph, title, QFont("sans", 12, QFont::Bold)));

	ui_graph->addGraph();
	ui_graph->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,7));
	ui_graph->graph(0)->setPen(QPen(QColor(40, 110, 255)));
	ui_graph->graph(0)->setLineStyle(QCPGraph::lsNone);
	ui_graph->graph(0)->setName("Current_cop_Fz");

	ui_graph->addGraph();
	ui_graph->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,7));
	ui_graph->graph(1)->setPen(QPen(QColor(255, 0, 0)));
	ui_graph->graph(1)->setLineStyle(QCPGraph::lsNone);
	ui_graph->graph(1)->setName("Reference_cop_Fz");

	ui_graph->addGraph();
	ui_graph->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssSquare,15));
	ui_graph->graph(2)->setPen(QPen(QColor(0, 0, 0)));
	ui_graph->graph(2)->setLineStyle(QCPGraph::lsNone);
	ui_graph->graph(2)->setName("Foot_L");

	ui_graph->addGraph();
	ui_graph->graph(3)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare,15));
	ui_graph->graph(3)->setPen(QPen(QColor(0, 0, 0)));
	ui_graph->graph(3)->setLineStyle(QCPGraph::lsNone);
	ui_graph->graph(3)->setName("Foot_R");

	ui_graph->xAxis->setLabel("Y  "+unit);
	ui_graph->yAxis->setLabel("X  "+unit);

	ui_graph->axisRect()->setupFullAxesBox();
	ui_graph->xAxis->setRange(min_value_x, max_value_x);
	ui_graph->xAxis->setRangeReversed(true);
	ui_graph->yAxis->setRange(min_value_y, max_value_y);
}
void MainWindow::graph_draw_update_none_line(QCustomPlot *ui_graph, double cur_value1, double cur_value2, double ref_value1, double ref_value2, double lf_value1, double lf_value2, double rf_value1, double rf_value2)
{
	QVector<double> c_1, c_2, r_11, r_22, lf_1, lf_2, rf_1, rf_2;

	c_1.append(cur_value1);
	c_2.append(cur_value2);

	r_11.append(ref_value1);
	r_22.append(ref_value2);

	lf_1.append(lf_value1);
	lf_2.append(lf_value2);

	rf_1.append(rf_value1);
	rf_2.append(rf_value2);

	ui_graph->graph(0)->setData(c_2, c_1);
	ui_graph->replot();
	ui_graph->update();

	ui_graph->graph(1)->setData(r_22, r_11);
	ui_graph->replot();
	ui_graph->update();

	ui_graph->graph(2)->setData(lf_2, lf_1);
	ui_graph->replot();
	ui_graph->update();

	ui_graph->graph(3)->setData(rf_2, rf_1);
	ui_graph->replot();
	ui_graph->update();

	c_1.clear();
	c_2.clear();
	r_11.clear();
	r_22.clear();
	lf_1.clear();
	lf_2.clear();
	rf_1.clear();
	rf_2.clear();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::graph_draw_none_line_flag(QCustomPlot *ui_graph, const QString title, const QString unit, double min_value_x, double max_value_x, double min_value_y, double max_value_y, int tick_count)
{
	ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
	ui_graph->legend->setVisible(false);
	legendFont.setPointSize(9); // and make a bit smaller for legend
	ui_graph->legend->setFont(legendFont);
	ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));
	// by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
	ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignRight);

	ui_graph->plotLayout()->insertRow(0);
	ui_graph->plotLayout()->addElement(0, 0, new QCPTextElement(ui_graph, title, QFont("sans", 12, QFont::Bold)));

	ui_graph->addGraph();
	ui_graph->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,7));
	ui_graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
	ui_graph->graph(0)->setLineStyle(QCPGraph::lsNone);
	ui_graph->graph(0)->setName("ROBOT");


	ui_graph->addGraph();
	ui_graph->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssSquare,15));
	ui_graph->graph(1)->setPen(QPen(QColor(0, 0, 0)));
	ui_graph->graph(1)->setLineStyle(QCPGraph::lsNone);

	ui_graph->addGraph();
	ui_graph->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssSquare,15));
	ui_graph->graph(2)->setPen(QPen(QColor(0, 0, 0)));
	ui_graph->graph(2)->setLineStyle(QCPGraph::lsNone);


	ui_graph->xAxis->setLabel("Y  "+unit);
	ui_graph->yAxis->setLabel("X  "+unit);

	ui_graph->axisRect()->setupFullAxesBox();
	ui_graph->xAxis->setRange(min_value_x, max_value_x);
	ui_graph->xAxis->setRangeReversed(true);
	ui_graph->yAxis->setRange(min_value_y, max_value_y);
}
void MainWindow::graph_draw_update_none_line_flag(QCustomPlot *ui_graph, double robot_x, double robot_y, double flag1_x, double flag1_y, double flag2_x, double flag2_y)
{
	QVector<double> r_11, r_22, f1_1, f1_2, f2_1, f2_2;

	r_11.append(robot_x);
	r_22.append(robot_y);

	f1_1.append(flag1_x);
	f1_2.append(flag1_y);

	f2_1.append(flag2_x);
	f2_2.append(flag2_y);

	ui_graph->graph(0)->setData(r_22, r_11);
	ui_graph->replot();
	ui_graph->update();

	ui_graph->graph(1)->setData(f1_2, f1_1);
	ui_graph->replot();
	ui_graph->update();

	ui_graph->graph(2)->setData(f2_2, f2_1);
	ui_graph->replot();
	ui_graph->update();

	r_11.clear();
	r_22.clear();
	f1_1.clear();
	f1_2.clear();
	f2_1.clear();
	f2_2.clear();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::graph_draw_top_view(QCustomPlot *ui_graph, const QString title, const QString unit, double min_value_x, double max_value_x, double min_value_y, double max_value_y, int tick_count)
{
	ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
	ui_graph->legend->setVisible(false);
	legendFont.setPointSize(9); // and make a bit smaller for legend
	ui_graph->legend->setFont(legendFont);
	ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));
	// by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
	ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignRight);

	ui_graph->plotLayout()->insertRow(0);
	ui_graph->plotLayout()->addElement(0, 0, new QCPTextElement(ui_graph, title, QFont("sans", 12, QFont::Bold)));

	ui_graph->addGraph();
	ui_graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
	ui_graph->graph(0)->setName("ROBOT");
	ui_graph->graph(0)->setLineStyle(QCPGraph::lsNone);
	ui_graph->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,7));

	for(int i = 1; i< 11; i++)
	{
		ui_graph->addGraph();
		ui_graph->graph(i)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssSquare,15));
		ui_graph->graph(i)->setPen(QPen(QColor(0, 0, 0)));
		ui_graph->graph(i)->setLineStyle(QCPGraph::lsNone);
	}


	ui_graph->xAxis->setLabel("Y  "+unit);
	ui_graph->yAxis->setLabel("X  "+unit);

	ui_graph->axisRect()->setupFullAxesBox();
	ui_graph->xAxis->setRange(min_value_x, max_value_x);
	ui_graph->xAxis->setRangeReversed(true);
	ui_graph->yAxis->setRange(min_value_y, max_value_y);
}

void MainWindow::graph_draw_update_top_view(QCustomPlot *ui_graph, double robot_x, double robot_y, double in_top_veiw_x[5], double in_top_veiw_y[5], double out_top_veiw_x[5], double out_top_veiw_y[5])
{
	QVector<double> f_x, f_y;

	r_1.append(robot_x);
	r_2.append(robot_y);

	ui_graph->graph(0)->setData(r_2, r_1);

	for(int num = 0; num < 5; num++)
	{
		f_x.append(in_top_veiw_x[num]);
		f_y.append(in_top_veiw_y[num]);
		ui_graph->graph(num+1)->setData(f_y, f_x);
		f_x.clear();
		f_y.clear();
	}
	for(int num = 0; num < 5; num++)
	{
		f_x.append(out_top_veiw_x[num]);
		f_y.append(out_top_veiw_y[num]);
		ui_graph->graph(num+6)->setData(f_y, f_x);
		f_x.clear();
		f_y.clear();
	}


	ui_graph->replot();
	ui_graph->update();

	//r_1.clear();
	//r_2.clear();
	f_x.clear();
	f_y.clear();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_graph_stop_button_1_clicked()
{
	dataTimer->stop();
}
void MainWindow::on_graph_start_button_1_clicked()
{
	dataTimer->start(0);
}
//initialize button
void MainWindow::on_initialize_ft_sensor_button_clicked()
{
	ft_init_msg.data = 1;
	qnode.diana_ft_init_pub.publish(ft_init_msg);
}
////// module on off /////////////////////////////

void MainWindow::on_online_walking_module_clicked() {
	module_msg.data = "online_walking_module";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_none_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}

void MainWindow::on_base_module_1_clicked() {
	module_msg.data = "base_module";
	qnode.module_on_off.publish(module_msg);
}

void MainWindow::on_init_module_call_clicked() {
	init_pose_msg.data = "init_pose";
	qnode.init_pose_pub.publish(init_pose_msg);
}
void MainWindow::on_init_offset_pose_button_clicked()
{
	init_pose_msg.data = "init_offset_pose";
	qnode.init_pose_pub.publish(init_pose_msg);
}
void MainWindow::on_offset_module_clicked() {
	module_msg.data = "offset_module";
	qnode.module_on_off.publish(module_msg);
}

void MainWindow::on_pose_module_clicked() {
	module_msg.data = "pose_module";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_motion_module_clicked() {
	module_msg.data = "motion_module";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_upper_body_module_button_clicked() {
	module_msg.data = "upper_body_module";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_arm_module_button_clicked() {
	module_msg.data = "arm_module";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_off_module_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_off_module_2_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_off_module_3_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_off_module_4_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}
//walking module button
void MainWindow::on_turn_left_clicked() {

	foot_step_command_msg.command = "turn left";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_turn_right_clicked() {

	foot_step_command_msg.command = "turn right";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_left_clicked() {

	foot_step_command_msg.command = "left";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_right_clicked() {
	foot_step_command_msg.command = "right";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}

void MainWindow::on_forward_clicked() {

	foot_step_command_msg.command = "forward";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_backward_clicked() {

	foot_step_command_msg.command = "backward";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}


void MainWindow::on_stop_clicked() {
	foot_step_command_msg.command = "stop";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}

void MainWindow::on_apply_data_clicked() {


	QString parameter_str;
	double  parameter_double = 0;


	parameter_str = ui.edit_step_num->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.step_num = parameter_double;

	parameter_str = ui.edit_step_length->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.step_length = parameter_double;

	parameter_str = ui.edit_side_step_length->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.side_step_length = parameter_double;

	parameter_str = ui.edit_step_angle_rad->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.step_angle_rad = parameter_double;

	parameter_str = ui.edit_step_time->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.step_time = parameter_double;

	// send message
	foot_step_command_msg.command = "stop";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);

}

void MainWindow::on_joint_feedback_gain_clicked()
{
	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("atest_gui") + "/config/joint_feedback_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	joint_feedback_gain_msg.request.updating_duration = doc["updating_duration"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_p_gain = doc["r_leg_hip_y_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_d_gain = doc["r_leg_hip_y_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_p_gain = doc["r_leg_hip_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_d_gain = doc["r_leg_hip_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_p_gain = doc["r_leg_hip_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_d_gain = doc["r_leg_hip_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_p_gain = doc["r_leg_an_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_d_gain = doc["r_leg_an_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_p_gain = doc["r_leg_an_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_d_gain = doc["r_leg_an_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_p_gain = doc["l_leg_hip_y_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_d_gain = doc["l_leg_hip_y_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_p_gain = doc["l_leg_hip_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_d_gain = doc["l_leg_hip_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_p_gain = doc["l_leg_hip_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_d_gain = doc["l_leg_hip_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_p_gain = doc["l_leg_an_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_d_gain = doc["l_leg_an_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_p_gain = doc["l_leg_an_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_d_gain = doc["l_leg_an_r_d_gain"].as<double>();


	qnode.joint_feedback_gain_client.call(joint_feedback_gain_msg);
}

void MainWindow::on_balance_param_apply_clicked()
{
	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("atest_gui") + "/config/balance_param.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}

	set_balance_param_msg.request.updating_duration = doc["updating_duration"].as<double>();


	set_balance_param_msg.request.balance_param.cob_x_offset_m = doc["cob_x_offset_m"].as<double>();
	set_balance_param_msg.request.balance_param.cob_y_offset_m = doc["cob_y_offset_m"].as<double>();

	//gain load //
	set_balance_param_msg.request.balance_param.foot_roll_gyro_p_gain = doc["foot_roll_gyro_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_gyro_d_gain = doc["foot_roll_gyro_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_gyro_p_gain = doc["foot_pitch_gyro_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_gyro_d_gain = doc["foot_pitch_gyro_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_roll_angle_p_gain = doc["foot_roll_angle_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_angle_d_gain = doc["foot_roll_angle_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_angle_p_gain = doc["foot_pitch_angle_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_angle_d_gain = doc["foot_pitch_angle_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_x_force_p_gain = doc["foot_x_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_x_force_d_gain = doc["foot_x_force_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_y_force_p_gain = doc["foot_y_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_y_force_d_gain = doc["foot_y_force_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_z_force_p_gain = doc["foot_z_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_z_force_d_gain = doc["foot_z_force_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_roll_torque_p_gain = doc["foot_roll_torque_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_torque_d_gain = doc["foot_roll_torque_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_pitch_torque_p_gain = doc["foot_pitch_torque_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_torque_d_gain = doc["foot_pitch_torque_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.roll_gyro_cut_off_frequency = doc["roll_gyro_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.pitch_gyro_cut_off_frequency = doc["pitch_gyro_cut_off_frequency"].as<double>();

	set_balance_param_msg.request.balance_param.roll_angle_cut_off_frequency = doc["roll_angle_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.pitch_angle_cut_off_frequency = doc["pitch_angle_cut_off_frequency"].as<double>();

	set_balance_param_msg.request.balance_param.foot_x_force_cut_off_frequency = doc["foot_x_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_y_force_cut_off_frequency = doc["foot_y_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_z_force_cut_off_frequency = doc["foot_z_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_torque_cut_off_frequency = doc["foot_roll_torque_cut_off_frequency"].as<double>();

	qnode.set_balance_param_client.call(set_balance_param_msg);
}

// control page gui /////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
//test
void MainWindow::on_left_motion_button_clicked()
{
	/*QString time_change_value_str;
	QString time_change_waist_value_str;
	double  time_change_value;
	double  time_change_waist_value;

	QString motion_center_value_str;
	double  motion_center_value;

	QString motion_edge_value_str;
	double  motion_edge_value;

	motion_center_value_str = ui.motion_center_value_text->text();
	motion_center_value = motion_center_value_str.toDouble();

	motion_edge_value_str = ui.motion_edge_value_text->text();
	motion_edge_value = motion_edge_value_str.toDouble();

	time_change_value_str = ui.time_change->text();
	time_change_value = time_change_value_str.toDouble();

	time_change_waist_value_str = ui.time_change_waist->text();
	time_change_waist_value = time_change_waist_value_str.toDouble();


	center_change_msg.turn_type = "pflug_bogen";
	center_change_msg.change_type = "left";

	center_change_msg.center_change = motion_center_value;
	center_change_msg.edge_change = motion_edge_value;

	center_change_msg.time_change = time_change_value;
	center_change_msg.time_change_waist = time_change_waist_value; // edge time

	qnode.center_change_pub.publish(center_change_msg);*/
}
void MainWindow::on_right_motion_button_clicked()
{
	/*	QString time_change_value_str;
	QString time_change_waist_value_str;
	double  time_change_value;
	double  time_change_waist_value;

	QString motion_center_value_str;
	double  motion_center_value;

	QString motion_edge_value_str;
	double  motion_edge_value;

	motion_center_value_str = ui.motion_center_value_text->text();
	motion_center_value = motion_center_value_str.toDouble();

	motion_edge_value_str = ui.motion_edge_value_text->text();
	motion_edge_value = motion_edge_value_str.toDouble();

	time_change_value_str = ui.time_change->text();
	time_change_value = time_change_value_str.toDouble();

	time_change_waist_value_str = ui.time_change_waist->text();
	time_change_waist_value = time_change_waist_value_str.toDouble();


	center_change_msg.turn_type = "pflug_bogen";
	center_change_msg.change_type = "right";

	center_change_msg.center_change = motion_center_value;
	center_change_msg.edge_change = motion_edge_value;
	center_change_msg.time_change = time_change_value;
	center_change_msg.time_change_waist = time_change_waist_value;

	qnode.center_change_pub.publish(center_change_msg);*/

}




//////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pflug_bogen_button_clicked()
{
	center_change_msg.turn_type = "pflug_bogen";
	qnode.center_change_pub.publish(center_change_msg);
}
void MainWindow::on_parallel_button_clicked()
{
	center_change_msg.turn_type = "parallel";
	qnode.center_change_pub.publish(center_change_msg);
}
void MainWindow::on_carving_button_clicked()
{
	center_change_msg.turn_type = "carving";
	qnode.center_change_pub.publish(center_change_msg);
}

void MainWindow::on_center_change_button_clicked()
{
	center_change_msg.change_type = "center_change";
	qnode.center_change_pub.publish(center_change_msg);
}
void MainWindow::on_edge_change_button_clicked()
{
	center_change_msg.change_type = "edge_change";
	qnode.center_change_pub.publish(center_change_msg);
}
void MainWindow::center_change(int k)
{
	QString time_change_value_str;
	QString time_change_waist_value_str;
	double  time_change_value;
	double  time_change_waist_value;
	QString slider_value_str_temp;

	slider_value_str_temp = QString::number(k*0.01);

	time_change_value_str = ui.time_change->text();
	time_change_value = time_change_value_str.toDouble();

	time_change_waist_value_str = ui.time_change_waist->text();
	time_change_waist_value = time_change_waist_value_str.toDouble();

	ui.slider_center_change_value->setText(slider_value_str_temp);

	center_change_msg.center_change = k*0.01;
	center_change_msg.time_change = time_change_value;
	center_change_msg.time_change_waist = time_change_waist_value;
	qnode.center_change_pub.publish(center_change_msg);
}

void MainWindow::edge_change(int k)
{
	QString time_change_value_str;
	double  time_change_value;
	QString slider_value_str_temp;

	slider_value_str_temp = QString::number(k*0.01);

	time_change_value_str = ui.time_change_edge->text();
	time_change_value = time_change_value_str.toDouble();

	ui.slider_edge_change_value->setText(slider_value_str_temp);

	center_change_msg.edge_change = k*0.01;
	center_change_msg.time_change = time_change_value;

	qnode.center_change_pub.publish(center_change_msg);
}
///////////////////////////////////////////////////////////////////////////////////////
////////////////// motion module ////////////////////////////////////////////////
void MainWindow::on_balance_on_clicked() {
	parse_gain_data();
	diana_balance_parameter_msg.updating_duration =  updating_duration;

	diana_balance_parameter_msg.cob_x_offset_m        = cob_x_offset_m;
	diana_balance_parameter_msg.cob_y_offset_m        = cob_y_offset_m;

	diana_balance_parameter_msg.foot_roll_gyro_p_gain = foot_roll_gyro_p_gain;
	diana_balance_parameter_msg.foot_roll_gyro_d_gain = foot_roll_gyro_d_gain;

	diana_balance_parameter_msg.foot_pitch_gyro_p_gain = foot_pitch_gyro_p_gain;
	diana_balance_parameter_msg.foot_pitch_gyro_d_gain = foot_pitch_gyro_d_gain;

	qnode.diana_balance_parameter_pub.publish(diana_balance_parameter_msg);
}
void MainWindow::on_balance_off_clicked() {
	diana_balance_parameter_msg.updating_duration =  updating_duration;

	diana_balance_parameter_msg.cob_x_offset_m        = 0;
	diana_balance_parameter_msg.cob_y_offset_m        = 0;

	diana_balance_parameter_msg.foot_roll_gyro_p_gain = 0;
	diana_balance_parameter_msg.foot_roll_gyro_d_gain = 0;

	diana_balance_parameter_msg.foot_pitch_gyro_p_gain = 0;
	diana_balance_parameter_msg.foot_pitch_gyro_d_gain = 0;

	qnode.diana_balance_parameter_pub.publish(diana_balance_parameter_msg);
}
void MainWindow::on_cop_control_leg_on_clicked() {
	parse_gain_data();
	diana_balance_parameter_msg.updating_duration =  updating_duration;

	diana_balance_parameter_msg.foot_copFz_p_gain = foot_copFz_p_gain;
	diana_balance_parameter_msg.foot_copFz_d_gain = foot_copFz_d_gain;

	qnode.diana_balance_parameter_pub.publish(diana_balance_parameter_msg);
}
void MainWindow::on_cop_control_leg_off_clicked() {
	diana_balance_parameter_msg.updating_duration =  updating_duration;

	diana_balance_parameter_msg.foot_copFz_p_gain = 0;
	diana_balance_parameter_msg.foot_copFz_d_gain = 0;

	qnode.diana_balance_parameter_pub.publish(diana_balance_parameter_msg);
}
//////////////////////////////////////////////////////////////////////
////////////////// upper body module /////////////////////////////////
void MainWindow::on_balance_on_waist_button_clicked()
{
	parse_gain_waist_data();
	diana_balance_parameter_waist_msg.updating_duration =  updating_duration_waist;

	diana_balance_parameter_waist_msg.waist_roll_gyro_p_gain = waist_roll_gyro_p_gain;
	diana_balance_parameter_waist_msg.waist_roll_gyro_d_gain = waist_roll_gyro_d_gain;

	diana_balance_parameter_waist_msg.waist_yaw_gyro_p_gain = waist_yaw_gyro_p_gain;
	diana_balance_parameter_waist_msg.waist_yaw_gyro_d_gain = waist_yaw_gyro_d_gain;

	qnode.diana_balance_parameter_waist_pub.publish(diana_balance_parameter_waist_msg);

}
void MainWindow::on_balance_off_waist_button_clicked()
{
	diana_balance_parameter_waist_msg.updating_duration =  updating_duration_waist;

	diana_balance_parameter_waist_msg.waist_roll_gyro_p_gain = 0;
	diana_balance_parameter_waist_msg.waist_roll_gyro_d_gain = 0;

	diana_balance_parameter_waist_msg.waist_yaw_gyro_p_gain = 0;
	diana_balance_parameter_waist_msg.waist_yaw_gyro_d_gain = 0;

	qnode.diana_balance_parameter_waist_pub.publish(diana_balance_parameter_waist_msg);

}
void MainWindow::on_balance_on_head_button_clicked()
{
	head_balance_msg.data = true;
	qnode.head_balance_pub.publish(head_balance_msg);

}
void MainWindow::on_balance_off_head_button_clicked()
{
	head_balance_msg.data = false;
	qnode.head_balance_pub.publish(head_balance_msg);
}
void MainWindow::on_cop_control_waist_on_clicked() {
	parse_gain_waist_data();
	diana_balance_parameter_waist_msg.updating_duration =  updating_duration_waist;

	diana_balance_parameter_waist_msg.waist_copFz_p_gain = waist_copFz_p_gain;
	diana_balance_parameter_waist_msg.waist_copFz_d_gain = waist_copFz_d_gain;

	qnode.diana_balance_parameter_waist_pub.publish(diana_balance_parameter_waist_msg);
}
void MainWindow::on_cop_control_waist_off_clicked() {
	diana_balance_parameter_waist_msg.updating_duration =  updating_duration_waist;

	diana_balance_parameter_waist_msg.waist_copFz_p_gain = 0;
	diana_balance_parameter_waist_msg.waist_copFz_d_gain = 0;

	qnode.diana_balance_parameter_waist_pub.publish(diana_balance_parameter_waist_msg);
}
////////////////// armmodule /////////////////////////////////
void MainWindow::on_balance_on_arm_button_clicked()
{
	parse_gain_arm_data();
	diana_balance_parameter_arm_msg.updating_duration     = updating_duration_arm;
	diana_balance_parameter_arm_msg.arm_roll_gyro_p_gain  = arm_roll_gyro_p_gain;
	diana_balance_parameter_arm_msg.arm_roll_gyro_d_gain  = arm_roll_gyro_d_gain;
	diana_balance_parameter_arm_msg.arm_pitch_gyro_p_gain = arm_pitch_gyro_p_gain;
	diana_balance_parameter_arm_msg.arm_pitch_gyro_d_gain = arm_pitch_gyro_d_gain;
	diana_balance_parameter_arm_msg.arm_yaw_gyro_p_gain   = arm_yaw_gyro_p_gain;
	diana_balance_parameter_arm_msg.arm_yaw_gyro_d_gain   = arm_yaw_gyro_d_gain;

	qnode.diana_balance_parameter_arm_pub.publish(diana_balance_parameter_arm_msg);

}

void MainWindow::on_balance_off_arm_button_clicked()
{
	diana_balance_parameter_arm_msg.updating_duration     = updating_duration_arm;
	diana_balance_parameter_arm_msg.arm_roll_gyro_p_gain  = 0;
	diana_balance_parameter_arm_msg.arm_roll_gyro_d_gain  = 0;
	diana_balance_parameter_arm_msg.arm_pitch_gyro_p_gain = 0;
	diana_balance_parameter_arm_msg.arm_pitch_gyro_d_gain = 0;
	diana_balance_parameter_arm_msg.arm_yaw_gyro_p_gain   = 0;
	diana_balance_parameter_arm_msg.arm_yaw_gyro_d_gain   = 0;

	qnode.diana_balance_parameter_arm_pub.publish(diana_balance_parameter_arm_msg);
}
void MainWindow::on_cop_control_arm_on_clicked() {
	parse_gain_arm_data();
	diana_balance_parameter_arm_msg.updating_duration = updating_duration_arm;

	diana_balance_parameter_arm_msg.arm_copFz_p_gain = arm_copFz_p_gain;
	diana_balance_parameter_arm_msg.arm_copFz_d_gain = arm_copFz_d_gain;

	qnode.diana_balance_parameter_arm_pub.publish(diana_balance_parameter_arm_msg);
}
void MainWindow::on_cop_control_arm_off_clicked() {

	diana_balance_parameter_arm_msg.updating_duration = updating_duration_arm;

	diana_balance_parameter_arm_msg.arm_copFz_p_gain = 0;
	diana_balance_parameter_arm_msg.arm_copFz_d_gain = 0;

	qnode.diana_balance_parameter_arm_pub.publish(diana_balance_parameter_arm_msg);
}
//////////////////////////////////////////////////////////////////////
// common ////////////////////////////////////////////////////////////

void MainWindow::on_control_on_all_clicked()
{
	on_balance_on_clicked();
	on_cop_control_leg_on_clicked();
	on_balance_on_waist_button_clicked();
	on_cop_control_waist_on_clicked();
	on_balance_on_arm_button_clicked();
	on_cop_control_arm_on_clicked();
}
void MainWindow::on_control_off_all_clicked()
{
	on_balance_off_clicked();
	on_cop_control_leg_off_clicked();
	on_balance_off_waist_button_clicked();
	on_cop_control_waist_off_clicked();
	on_balance_off_arm_button_clicked();
	on_cop_control_arm_off_clicked();
}

void MainWindow::parse_gain_data()
{
	updating_duration = 0.0;
	cob_x_offset_m = 0.0;
	cob_y_offset_m = 0.0;
	foot_roll_gyro_p_gain = 0.0;
	foot_roll_gyro_d_gain = 0.0;
	foot_pitch_gyro_p_gain = 0.0;
	foot_pitch_gyro_d_gain = 0.0;
	foot_copFz_p_gain = 0.0;
	foot_copFz_d_gain = 0.0;

	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("atest_gui") + "/config/leg_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	// time load //
	updating_duration = doc["updating_duration"].as<double>();

	// offset load //
	cob_x_offset_m = doc["cob_x_offset_m"].as<double>();
	cob_y_offset_m = doc["cob_y_offset_m"].as<double>();

	//gain load //
	foot_roll_gyro_p_gain = doc["foot_roll_gyro_p_gain"].as<double>();
	foot_roll_gyro_d_gain = doc["foot_roll_gyro_d_gain"].as<double>();

	foot_pitch_gyro_p_gain = doc["foot_pitch_gyro_p_gain"].as<double>();
	foot_pitch_gyro_d_gain = doc["foot_pitch_gyro_d_gain"].as<double>();

	foot_copFz_p_gain = doc["foot_copFz_p_gain"].as<double>();
	foot_copFz_d_gain = doc["foot_copFz_d_gain"].as<double>();
}

void MainWindow::parse_gain_waist_data()
{
	updating_duration_waist = 0.0;
	waist_roll_gyro_p_gain = 0.0;
	waist_roll_gyro_d_gain = 0.0;
	waist_yaw_gyro_p_gain = 0.0;
	waist_yaw_gyro_d_gain = 0.0;
	waist_copFz_p_gain = 0.0;
	waist_copFz_d_gain = 0.0;

	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("atest_gui") + "/config/upper_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	// time load //
	updating_duration_waist = doc["updating_duration"].as<double>();

	//gain load //
	waist_roll_gyro_p_gain  = doc["waist_roll_gyro_p_gain"].as<double>();
	waist_roll_gyro_d_gain  = doc["waist_roll_gyro_d_gain"].as<double>();
	waist_yaw_gyro_p_gain   = doc["waist_yaw_gyro_p_gain"].as<double>();
	waist_yaw_gyro_d_gain   = doc["waist_yaw_gyro_d_gain"].as<double>();
	waist_copFz_p_gain = doc["waist_copFz_p_gain"].as<double>();
	waist_copFz_d_gain = doc["waist_copFz_d_gain"].as<double>();

}

void MainWindow::parse_gain_arm_data()
{
	updating_duration_arm  = 0.0;
	arm_roll_gyro_p_gain   = 0.0;
	arm_roll_gyro_d_gain   = 0.0;
	arm_pitch_gyro_p_gain  = 0.0;
	arm_pitch_gyro_d_gain  = 0.0;
	arm_yaw_gyro_p_gain    = 0.0;
	arm_yaw_gyro_d_gain    = 0.0;
	arm_copFz_p_gain = 0.0;
	arm_copFz_d_gain = 0.0;

	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("atest_gui") + "/config/arm_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	// time load //
	updating_duration_arm = doc["updating_duration"].as<double>();

	//gain load //
	arm_roll_gyro_p_gain   = doc["arm_roll_gyro_p_gain"].as<double>();
	arm_roll_gyro_d_gain   = doc["arm_roll_gyro_d_gain"].as<double>();
	arm_pitch_gyro_p_gain  = doc["arm_pitch_gyro_p_gain"].as<double>();
	arm_pitch_gyro_d_gain  = doc["arm_pitch_gyro_d_gain"].as<double>();
	arm_yaw_gyro_p_gain    = doc["arm_yaw_gyro_p_gain"].as<double>();
	arm_yaw_gyro_d_gain    = doc["arm_yaw_gyro_d_gain"].as<double>();
	arm_copFz_p_gain       = doc["arm_copFz_p_gain"].as<double>();
	arm_copFz_d_gain       = doc["arm_copFz_d_gain"].as<double>();
}
// decision_module
void MainWindow::on_decision_process_on_button_clicked()
{
	ready_check_msg.data = true;
	qnode.ready_check_pub.publish(ready_check_msg);

}
void MainWindow::on_decision_process_off_button_clicked()
{
	ready_check_msg.data = false;
	qnode.ready_check_pub.publish(ready_check_msg);
}

}  // namespace atest_gui

