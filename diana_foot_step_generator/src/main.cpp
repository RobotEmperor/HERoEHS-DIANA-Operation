/*
 * main.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: jaysong
 */

#include "diana_foot_step_generator/message_callback.h"

int main( int argc , char **argv )
{
    ros::init( argc , argv , "diana_foot_step_generator" );

    initialize();

    ros::spin();
    return 0;
}



