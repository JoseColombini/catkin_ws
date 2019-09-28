
#include <cstdio>
#include "math.h"
#include <cstring>
#include <iostream>
#include <ctime>
#include <stdlib.h>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>


class UAV {
//Atributos
public:
  ros::NodeHandle nh;
    ros::Rate rate;
  //Publisher
  ros::Publisher local_position_pub; //publ ica para onde o drone tem que ir
  ros::Publisher Stay_pub; //publica se o drone deve ficar parado
  //Subscriber
  ros::Subscriber local_atual; //subs para saber onde o drone esta
  ros::Subscriber state_status_subscribe; //identifica o status do drone
  ros::Subscriber Stay_sub; //verifica se o dorne esta esperando ordem
  ros::Subscriber aligned_sub;
  //Services
  //so acertar os services no construtor
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  //Drone atributes to interac with pub and Subscriber
  geometry_msgs::PoseStamped drone_pose; //onde o drone esta
  geometry_msgs::PoseStamped goal_pose; //para o drone vai
  geometry_msgs::TwistStamped goal_vel; //qual a velocidade do drone vai
  mavros_msgs::State drone_state; //estado do drone
  mavros_msgs::CommandBool drone_stay_pub; //mensagem que ordena se o drone vai ficar parado esperando ordem
  mavros_msgs::CommandBool dorne_stay_sub; //verifica se o drone esta parado esperando ordem
  std_msgs::String is_aligned;
  mavros_msgs::CommandBool arming_bool;
//Metodos
//  UAV (); //fazer services
  //callbacks
  //void state_callback(const mavros_msgs::State& state_data);
  //void local_callback(const geometry_msgs::PoseStamped& local);
  //void stay_callback(const mavros_msgs::CommandBool& stay_here);
  //setters
  //void set_position(float x, float y, float z);
  //set_vel esta meio perdido aqui
  //void set_vel(float x, float y, float z); //acertar a questao das velocidade angulares
  //void set_stay(bool x);
  //safity
  //bool chegou();
  //missions
  //std_msgs::String takeoff(float height);
  //std_msgs::String RTL();



/*********Construtor*************/
UAV () {

  this->rate(const 20.0);
  //Publisher
  local_position_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("/mavros/setpoint_position/local", 10);
  Stay_pub = nh.advertise<mavros_msgs::CommandBool>
    ("dronecontrol/stay", 1);
  //Subscribers
  local_atual = nh.subscribe
    ("mavros/local_position/pose", 0, &UAV::local_callback, this);
  state_status_subscribe = nh.subscribe
    ("mavros/state", 0, &UAV::state_callback, this);
  aligned_sub = nh.subscribe
    ("dronecontrol/stay", 1, &UAV::stay_callback, this);
  local_atual = nh.subscribe("align_reference/aligned", 0, &UAV::aligned_callback, this);

  //Services
  this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
  this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");
}


/*********Callbacks***********/
void aligned_callback(const std_msgs::String& data) {
  this->is_aligned = data;
}

void state_callback(const mavros_msgs::State& state_data) {
  this->drone_state = state_data;
}

void local_callback(const geometry_msgs::PoseStamped& local) {
  this->drone_pose.pose.position.x = local.pose.position.x;
  this->drone_pose.pose.position.y = local.pose.position.y;
  this->drone_pose.pose.position.z = local.pose.position.z;
}

void stay_callback(const mavros_msgs::CommandBool& stay_here) {
  this->dorne_stay_sub = stay_here;
  while(this->dorne_stay_sub.response.success) {
    set_position(this->goal_pose.pose.position.x,
                 this->goal_pose.pose.position.y,
                 this->goal_pose.pose.position.z);
  }
}

/**************Setters****************/
void set_position(float x, float y, float z) {
  this->goal_pose.pose.position.x = x;
  this->goal_pose.pose.position.y = y;
  this->goal_pose.pose.position.z = z;
  local_position_pub.publish(goal_pose);
}


void set_vel(float x, float y, float z) {
  this->goal_vel.twist.linear.x = x;
  this->goal_vel.twist.linear.y = y;
  this->goal_vel.twist.linear.z = z;

  //ainda definir de forma correta isto aqui as angulares
  this->goal_vel.twist.angular.x = 0; //roll
  this->goal_vel.twist.angular.y = 0; //pitch
  this->goal_vel.twist.angular.z = 0; //yaw

}

void set_stay(bool x) {
  this->drone_stay_pub.request.value = x;
  Stay_pub.publish(this->drone_stay_pub);
}


bool chegou() {
  if (math::abs(this->goal_pose.pose.position.x - this->drone_pose.pose.position.x) && math::abs(this->goal_pose.pose.position.y - this->drone_pose.pose.position.y) && cmath::abs(this->goal_pose.pose.position.z - this->drone_pose.pose.position.z)) {
    return true;
  }
  else
    return false;
}


/************Funciontions***************/
std_msgs::String takeoff(float height) {
  while (this->dorne_stay_sub.response.success)
    this->set_stay(0);

  this->arming_bool.request.value = true;
  mavros_msgs::SetMode setting_mode;
  float velocity = 0.3;
  float part = velocity/60.0;

  while (!this->drone_state.armed && (this->drone_state.mode != "OFFBOARD")) {
    //rospy.logwarn n sei oq fazer
    this->arming_client.call(arming_bool);
    this->rate.sleep();
    this->set_mode_client.call() = "OFFBOARD";

  }

  time_t init_time;
  init_time = time(NULL);
  int t = 0;



  while (ros::ok() && !UAV::chegou()) {
    //n sei fazer loginfo
    //ROS_INF
    this->set_position(0, 0, t);
    t += part;
  }
  this->set_stay(1);
}


std_msgs::String RTL(){
  time_t init_time;
  float velocity = 0.3;
  float ds = velocity/60.0;

  this->rate.sleep();

  float height = this->drone_pose.pose.position.z;
  printf("%f\n", height);

  this->set_position(0, 0, height);
  this->rate.sleep();

  while (!this->chegou()) {
    this->set_position(0, 0, height);
    this->rate.sleep();
  }

  float t = ds;
  this->set_position(0, 0, height - t);
  this->rate.sleep();

  while (!this->drone_pose.pose.position.z < 0.15) {
    printf("%f\n", height);
    set_position(0, 0, height - t);
    if(!this->chegou())
      t += ds;
    this->rate.sleep();
  }
  set_position (0, 0, -1);
  this->rate.sleep();
  this->arm.request.value = false;
  return "succeeded";

}
};
