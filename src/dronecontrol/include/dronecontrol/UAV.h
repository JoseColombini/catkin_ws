/*#ifndef UAV_H
#define UAV_H

#include <cstdio>
#include <cmath>
#include <cstring>
#include <iostream>

#include "mavros_msgs.h"
#include "ros/ros.h"
#include "mavros_msgs/srv.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/State.h"


class UAV {
//Atributos
public:
  ros::NodeHandle nh;
  //Publisher
  ros::Publisher local_position_pub;
  //Subscriber
  ros::Subscriber local_atual;
  ros::Subscriber state_status_subscribe;
  //Services
  //so acertar os services no construtor
  ros::ServiceProxy arm;
  ros::ServiceProxy set_mode;
  //Drone atributes to interac with pub and Subscriber
  geometry_msgs::PoseStamped drone_pose;
  geometry_msgs::PoseStamped goal_pose;
  geometry_msgs::TwistStamped goal_vel;
  mavros_msgs::State drone_state;

//Metodos
  UAV (); //fazer services
  //callbacks
  void state_callback(const mavros_msgs::State& state_data);
  void local_callback(const geometry_msgs::PoseStamped& local);
  //setters
  void set_position(float x, float y, float z);
  //set_vel esta meio perdido aqui
  void set_vel(float x, float y, float z); //acertar a questao das velocidade angulares
  //safity
  bool chegou();
  //missions
  string takeoff(float height);
  string RTL();
};

#endif
*/
