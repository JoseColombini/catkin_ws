#include <stdlib.h>
#include "math.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>


std_msgs::Bool interrupted;
std_msgs::Bool node_message;
std_msgs::Bool aligned_message;
double position[3];
geometry_msgs::PoseStamped pub;


void callback_fcu_pose (const geometry_msgs::PoseStamped msg) {
  position[0] = msg.pose.position.x;
  position[1] = msg.pose.position.y;
  position[2] = msg.pose.position.z;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "align");
  ros::NodeHandle nh;
  ros::Rate rate(20.0);
  ros::Publisher interrupted_pub = nh.advertise<std_msgs::Bool>
        ("pack/node", 1);
  ros::Publisher set_position_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
  ros::Subscriber set_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10, callback_fcu_pose);

    interrupted.data = true;
    for(int i; i < 1000; i++) {
    printf("%f / %f / %f\n", position[0], position[1], position[2]);
    pub.pose.position.x = position[0];
    pub.pose.position.y = position[1];
    pub.pose.position.z = position[2];
    set_position_pub.publish(pub);
    interrupted_pub.publish(interrupted);
    ros::spinOnce();
    rate.sleep();
}
}
