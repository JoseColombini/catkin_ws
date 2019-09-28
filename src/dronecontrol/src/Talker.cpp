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

int main(int argc, char **argv) {

  ros::init(argc, argv, "Inter");
  ros::NodeHandle nh;
  ros::Rate rate(20.0);
  ros::Publisher interrupted_pub = nh.advertise<std_msgs::Bool>
        ("pack/aligned", 1);
    interrupted.data = true;
    while (true) {
      /* code */

    interrupted_pub.publish(interrupted);
    ros::spinOnce();
    rate.sleep();
}
}
