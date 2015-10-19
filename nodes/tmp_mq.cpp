/*
*	File:	tmp_mq.cpp
*
*	Date:	08/06/2015
*
*	Author:	Jenni Lam
*	Email:	jennil.lam@mail.utoronto.ca
*		
*	Description:	creates fake pollution sensor data and publishes it on the *					MQ topic													*/

#include <ros/ros.h>
#include <std_msgs/Int8.h>

int main(int argc, char **argv) {

	int count = 0;	
	int data = 15;
	int increment = 5;
	std_msgs::Int8 gas;

	// initialize node
	ros::init(argc, argv, "tmp_mq");
	ros::NodeHandle n;

	// initialize publisher
	ros::Publisher mq2_pub = n.advertise<std_msgs::Int8>("MQ", 5);
	ROS_INFO("Publishers and subscribers intialized");

	ros::Rate loop_rate(20);

	while (ros::ok()) {

		// publish fake data
		gas.data = data;	
		mq2_pub.publish(gas);

		// update fake data
		if (count == 50) {
			count = 0;
			data = data + increment;
			if (data >= 100) {
				increment = -5;
			} else if (data <= 15) {
				increment = 5;
			} // if
		} // if
		count++;

		ros::spinOnce();
		loop_rate.sleep();
	} // while
	
	return 0;
} // main
