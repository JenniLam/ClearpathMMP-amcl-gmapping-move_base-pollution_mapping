/*
*	File:	pmapping.cpp
*
*	Date:	08/06/2015
*
*	Author:	Jenni Lam
*	Email:	jennil.lam@mail.utoronto.ca
*		
*	Description:	reads the current map of the surroundings and adds visual 
*					representations of pollution readings to the current location
*					of the robot													*/


#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <math.h>

int total_length = 0;					// total length of map array
int obsmap_prev = 0;					// previous length of obstacle map array
int gas = 0;							// gas sensor reading
int x = 0;								// current x index
int y = 0;								// current y index

int top = 0;							// top limit of pollution map
int bot = 0;							// bottom limit of pollution map
int left = 0;							// left limit of pollution map
int right = 0;							// right limit of pollution map

bool gasCalled = false;					// true once gasCallback has been called for the first time
bool mapCalled = false;					// true once mapCallback has been called for the first time
bool poseCalled = false;				// true once poseCallback has been called for the first time

std::vector <int> obsmap;				// obstacle map in row-major order
std::vector <int> pmap;					// pollution map in row-major order
std::vector <int> tmp;					// temporary pollution map in row-major order
std_msgs::Header header;				// header for the pollution map
geometry_msgs::Pose pose;				// current pose of the robot
geometry_msgs::Pose origin;				// origin pose of map
cv_bridge::CvImage im;					// pollution map as image

std_msgs::Float32 resolution;			// map resolution
std_msgs::UInt32 width;					// map width
std_msgs::UInt32 height;				// map height


/* stores data from gas sensors									
*  msg is an array containing data from Arduino 				*/
void gasCallback(const std_msgs::Int8 msg) {

	gasCalled = true;
	gas = (int)msg.data;

} // gasCallback


/* stores data from current map									
*  msg is an OccupancyGrid containing obstacle data 			*/
void mapCallback(const nav_msgs::OccupancyGrid msg) {

	mapCalled = true;
	header = msg.header;

	width.data = msg.info.width;
	height.data = msg.info.height;
	resolution.data = msg.info.resolution;
	total_length = width.data*height.data;

	// records current obstacle map 
	// reflected horizontally to match rviz visual of current map
	for (int i = width.data; i <= total_length+width.data; i = i+width.data) {
		for (int j = 1; j <= width.data; j++) {
			if (i - width.data + j - 1 >= obsmap_prev) {
				obsmap.push_back(msg.data[i-j]);
			} else {
				obsmap[i - width.data + j - 1] = msg.data[i-j];
			} // else
		} // for
	} // for

	origin.position = msg.info.origin.position;
	origin.orientation = msg.info.origin.orientation;
	obsmap_prev = total_length;

} // mapCallback


/* stores robot pose data										
*  msg is PoseWithCovarianceStamped containing data from 		
*  robot_pose_ekf 												*/
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {

	poseCalled = true;
	pose.position = msg.pose.pose.position;
	pose.orientation = msg.pose.pose.orientation;

} // poseCallback


/* adds new data to the pollution map							
*  pmap is the pollution map									
*  the gas vector contains current gas sensor data 				*/
void addData() {

	int row = 0;

	// adds sensor values for current origin of the robot and for the cells
	// in its vicinity
	for (int i = 0; i < total_length+1; i = i+width.data) {
		for (int j = 0; j < width.data; j++) {
			row = i/width.data;
			if (std::abs(row - x) + std::abs(j - y) <= 3) {
				if (pmap[i+j] == 0) {
					pmap[i+j] = gas;
				} else {
					pmap[i+j] = (int)(((float)pmap[i+j] + (float)gas)/2);
				} // else
			} // if
		} // for
	} // for

} // addData


/* gets current grid coordinates of the center of robot in 		
*  "matrix" indices												
*  pose is the pose of the robot								
*  x, y are the indices of the "matrix"							*/
void getCoordinates() {

	y = height.data - (pose.position.x - origin.position.x)/resolution.data;
	x = (pose.position.y - origin.position.y)/resolution.data;

} // getCoordinates


/* finds acceptable limits for the pollution map image			
*  pollution map image does not need to be the size of the map	
*  and often can be much smaller								
*  obsmap is the obstacle map of the robot						
*  top, left, right, bot are the index limits of the "matrix"	*/
void findLimits() {
	bool isFirst = true;
	bool leftmost = false;

	top = 0;
	bot = 0;
	left = 0;
	right = 0;

	// find limits based on current known territory in obstacle map
	for (int i = 0; i < height.data; i++) {
		for (int j = 0; j < width.data; j++) {
			if (obsmap[i*width.data + j] != -1) {
				if (isFirst) {
					isFirst = false;
					top = i;
				} // if
				if (!leftmost) {
					leftmost = true;
					left = j;
				} // if
				if (leftmost && j < left) {
					left = j;
				} // if
				if (j > right) {
					right = j;
				} // if
				bot = i;
			} // if

		} // for
	} // for

	// add padding on sides of map
	top -= 10;
	bot += 10;
	left -= 10;
	right += 10;

	// ensure robots current position is included in map
	if (x - left <= 5) {
		left = x - 10;
	} // if
	if (right - x <= 5) {
		right = x + 10;
	} // if
	if (y - top <= 5) {
		top = y - 10;
	} // if
	if (bot - y <= 5) {
		bot = y + 10;
	} // if

	// ensure current limits fall inside map
	if (top < 0) {
		top = 0;
	} // if
	if (bot > height.data) {
		bot = height.data;
	} // if
	if (left < 0) {
		left = 0;
	} // if
	if (right > width.data) {
		right = width.data;
	} // if

} // findLimits


int main(int argc, char **argv) {

	float green = 0;						// green value in bgr image
	CvMat* im_ptr;							// pointer for mat containing pollution map
	cv::Scalar color = cv::Scalar(0,0,0);	// color to of current image cell

	// initialize node
	ros::init(argc, argv, "pmapping");
	ros::NodeHandle n;

	// initialize subscribers and publishers
	ros::Subscriber p_sub = n.subscribe("/MQ", 5, gasCallback);
	ros::Subscriber map_sub = n.subscribe("/map", 5, mapCallback); 
	ros::Subscriber pose_sub = n.subscribe("/robot_pose_ekf/odom_combined", 5, poseCallback); 
	ros::Publisher im_pub = n.advertise<sensor_msgs::Image>("/images/pmap", 5);
	ROS_INFO("Publishers and subscribers intialized");

	// refresh until first callbacks have been made
	ros::Rate loop_rate(30);
	while(!gasCalled || !mapCalled || !poseCalled) {
		ros::spinOnce();
		loop_rate.sleep();
	} // while

	// create pollution map of proper size
	im.encoding = "bgr8";
	for (int i = 0; i < total_length; i++) {
		pmap.push_back(0);
	} // for
	ROS_INFO("Pollution map initialized");
	ROS_INFO("Pollution mapping started");

	while (ros::ok()) {

		total_length = width.data*height.data;
		getCoordinates();
		addData();		
		findLimits();

		// create mat of pollution map and fill in colors based on readings
		im_ptr = cvCreateMat(bot-top+1, right-left+1, CV_8UC3);
		for (int i = top; i < bot; i++) {
			for (int j = left; j < right; j++) {
				if (i == x && y == j) {
					color = cv::Scalar(0,0,200);
				} else if (obsmap[i*width.data + j] == 100) {
					color = cv::Scalar(0,0,0);
				} else if (pmap[i*width.data+j] == 0) {
					color = cv::Scalar(255,255,255);
				} else {
					if (pmap[i*width.data+j] < 50) {
						green = 200;
					} else if (pmap[i*width.data+j] < 115) {
						green = 150;
					} else {
						green = 0;
					} // else
					color = cv::Scalar(255.0 - float(pmap[i*width.data+j]*2), green, 0);
				} // else
				cvSet2D(im_ptr, i-top, j-left, color);
			} // for
		} // for

		im.image = im_ptr;

		im.header = header;
		im_pub.publish(im.toImageMsg());

		ros::spinOnce();
		loop_rate.sleep();
	} // while
	
	return 0;
} // main
