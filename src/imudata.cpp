#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Quaternion.h"
#include <gsl/gsl_matrix.h>
#include "aauship_control/ADIS16405.h"
#include "sensor_msgs/Imu.h"
#define NR_DATA 9
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

ros::Publisher imu_data_pub;
float data[NR_DATA] = {0, 0, 0, 0, 0, 0, 0, 0, 0};



void Quaternion(const aauship_control::ADIS16405::ConstPtr& input)
{
	data[0] = (input->xgyro);
	data[1] = (input->ygyro);
	data[2] = (input->zgyro);
	data[3] = (input->xaccl);
	data[4] = (input->yaccl);
	data[5] = (input->zaccl);
	data[6] = (input->xmagn);
	data[7] = (input->ymagn);
	data[8] = (input->zmagn);

	tf::Quaternion q = tf::createQuaternionFromRPY(data[6], data[7], data[8]);
	std::cout << "euler angle is " << data[6] << " " << data[7]<< " " << data[8] <<std::endl;
	std::cout << "Quaternion is " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<std::endl;


	ROS_INFO_STREAM(q);
	//Publish the states
	sensor_msgs::Imu imu_data_msg;
	imu_data_msg.orientation.x = q[0];
	imu_data_msg.orientation.y = q[1];
	imu_data_msg.orientation.z = q[1];
	imu_data_msg.orientation.w = q[1];
	imu_data_msg.orientation_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	imu_data_msg.angular_velocity.x = data[6];
	imu_data_msg.angular_velocity.y = data[7];
	imu_data_msg.angular_velocity.z = data[8];
	imu_data_msg.angular_velocity_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	imu_data_msg.linear_acceleration.x = data[3];
	imu_data_msg.linear_acceleration.y = data[4];
	imu_data_msg.linear_acceleration.z = data[5];
	imu_data_msg.linear_acceleration_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	imu_data_pub.publish(imu_data_msg);

}


int main(int argc, char **argv)
{

        ros::init(argc, argv, "imudata");
        ros::NodeHandle n;
        ros::Subscriber imu_update = n.subscribe("/imu",1000,Quaternion);
        imu_data_pub = n.advertise<sensor_msgs::Imu>("/example/imu", 1);

	ros::spin();
return 0;
}
