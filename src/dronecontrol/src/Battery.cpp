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
#include <sensor_msgs/BatteryState.h>

double percent;
bool end;
void battery_callback(const sensor_msgs::BatteryState bat) {
  percent = bat.percentage;
}
void mission_callback(const std_msgs::Bool msg) {
  end = msg.data;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "battery");

  ros::NodeHandle nh;
    ros::Rate rate (20.0);
  ros::Subscriber bat_percent_sub = nh.subscribe<sensor_msgs::BatteryState>
      ("mavros/battery", 1, battery_callback);
  ros::Subscriber end_mission_sub = nh.subscribe<std_msgs::Bool>
      ("dronecontrol/end_mission", 1, mission_callback);
    while (!end) {
    //  printf("%d\n",end);
      printf("%f\n", percent);
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
