/*
 * message_callback.h
 *
 *  Created on: Feb 28, 2018
 *      Author: jaysong
 */

#ifndef DIANA_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_
#define DIANA_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <yaml-cpp/yaml.h>

#include "diana_foot_step_generator/FootStepCommand.h"
//#include "diana_foot_step_generator/Step2DArray.h"

#include "robotis_controller_msgs/StatusMsg.h"
#include "diana_online_walking_module_msgs/RobotPose.h"
#include "diana_online_walking_module_msgs/GetReferenceStepData.h"
#include "diana_online_walking_module_msgs/AddStepDataArray.h"
#include "diana_online_walking_module_msgs/StartWalking.h"
#include "diana_online_walking_module_msgs/SetBalanceParam.h"
#include "diana_online_walking_module_msgs/IsRunning.h"
#include "diana_online_walking_module_msgs/RemoveExistingStepData.h"

#include "robotis_foot_step_generator.h"


void initialize(void);

void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);

void walkingCommandCallback(const diana_foot_step_generator::FootStepCommand::ConstPtr& msg);
//void step2DArrayCallback(const diana_foot_step_generator::Step2DArray::ConstPtr& msg);

bool isRunning(void);



#endif /* DIANA_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_ */
