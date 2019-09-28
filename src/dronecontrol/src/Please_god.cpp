#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>


geometry_msgs::PoseStamped goal_pose;
mavros_msgs::State current_state;

std_msgs::String message;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



void set_position (float x, float y, float z, ros::Publisher position_pub) {

  goal_pose.pose.position.x = x;
  goal_pose.pose.position.y = y;
  goal_pose.pose.position.z = z;

  position_pub.publish(goal_pose);
}


int main(/*int argc, char **argv*/) {
    int argc;
    char **argv;
    float altitude_desejada = 2.0;

    ros::init(argc, argv, "integracao_controle");

    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher position_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher vel_pos_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

            ros::Rate rate(20.0);

            /* A pilha de vôo do PX4 possui um intervalo de 500ms entre dois comandos offboard. Se esse intervalo for excedido, o programa de comando irá voltar para o último estado da aeronave antes de entrar no modo offboard! */

            while(ros::ok() && !current_state.connected){

                ros::spinOnce();
                rate.sleep();
            }

        	/* Antes de publicarmos algo, nós esperamos a conexão a ser estabelecida entre a MAVROS e o autopilot. Esse loop se encerrará quando a mensagem for recebida! */

            goal_pose.pose.position.x = 0;
            goal_pose.pose.position.y = 0;
            goal_pose.pose.position.z = 0;

            //send a few setpoints before starting

            for(int i = 100; ros::ok() && i > 0; --i){

                local_pos_pub.publish(goal_pose);

                ros::spinOnce();

                rate.sleep();
            }

            /* Antes de entrarmos no modo offboard, você tem que ter iniciado os setpoints de transmissão. Caso contrário, a escolha do modo de vôo será rejeitada (próx. passo). Aqui, o número 100 foi uma escolha arbitrária */

              mavros_msgs::SetMode offb_set_mode;
              offb_set_mode.request.custom_mode = "OFFBOARD";

              mavros_msgs::CommandBool arm_cmd;
              arm_cmd.request.value = true;

              ros::Time last_request = ros::Time::now();

              goal_pose.pose.position.x = 0.0;
              goal_pose.pose.position.y = 0.0;
              goal_pose.pose.position.z = altitude_desejada;

              local_pos_pub.publish(goal_pose);


              while(ros::ok()){

          	if( current_state.mode != "OFFBOARD" &&
                      (ros::Time::now() - last_request > ros::Duration(5.0))){
                      if( set_mode_client.call(offb_set_mode) &&
                          offb_set_mode.response.mode_sent){
                          ROS_INFO("\n[ INFO ] Offboard enabled");
                      }
                      last_request = ros::Time::now();
                  }

          	else {
                      if( !current_state.armed &&
                          (ros::Time::now() - last_request > ros::Duration(5.0))){
                          if( arming_client.call(arm_cmd) &&
                              arm_cmd.response.success){
                              ROS_INFO("\n[ WARN ] Skyrats MAV armed");
                          }
                          last_request = ros::Time::now();
                      }
                  }
            while (true) {
              /* code */

          	local_pos_pub.publish(goal_pose);
            ros::spinOnce();
            rate.sleep();
          }
        }
  return 0;
}
