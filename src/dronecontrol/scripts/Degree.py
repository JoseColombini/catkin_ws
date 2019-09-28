#!/usr/bin/env python

import rospy
import mavros_msgs
import std_msgs
from std_msgs.msg import Float64
from mavros_msgs import srv
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
#from std_msgs.msg import Float64
import time

goal_pose = PoseStamped()
current_state = State()
local = PoseStamped()
Drone_local = PoseStamped()
Degree_init = Float64()

def rotation (x):
    global y
    y = x

def set_position(x, y, z):
    global goal_pose
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z
    local_position_pub.publish(goal_pose)

def state(state_data):
    global current_state
    current_state = state_data


rospy.init_node('Vel_Control_Node', anonymous = True)

rate = rospy.Rate(20)

local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 100)

state_status_subscribe = rospy.Subscriber('/mavros/state', State, state)

arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

Degree_init = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, rotation)

for i in range(300):
    local_position_pub.publish(goal_pose)
    rate.sleep()

rospy.loginfo("[ROS] SETUP CONCLUIDO")

grau_inicial = y

while not rospy.is_shutdown():
    if current_state != "OFFBOARD" or not current_state.armed:
        arm (True)
        set_mode(custom_mode = "OFFBOARD")


    print(str(current_state.mode))

    if current_state.armed == True:
        rospy.loginfo("DRONE ARMED")

    if current_state.mode == "OFFBOARD":
        rospy.loginfo('OFFBOARD mode setted')

    print ("graus: ", y)
