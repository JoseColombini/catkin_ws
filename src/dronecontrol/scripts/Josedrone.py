#!/usr/bin/env python

import rospy
import mavros_msgs
from mavros_msgs import srv
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
import time

print('importou')

goal_pose = PoseStamped()
current_state = State()
local = PoseStamped()
Drone_pose = PoseStamped()


print('criou variais')

def set_position(x, y, z):
    global goal_pose
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z
    local_position_pub.publish(goal_pose)

def state(state_data):
    global current_state
    current_state = state_data

def drone_position(local):
    global Drone_pose

    Drone_pose.pose.position.x = local.pose.position.x
    Drone_pose.pose.position.y = local.pose.position.y
    Drone_pose.pose.position.z = local.pose.position.z

def secure_accuracy (x,y):
    if (abs(x.pose.position.z - y.pose.position.z) < 0.1) and (abs(x.pose.position.y - y.pose.position.y) < 0.1) and (abs(x.pose.position.x - y.pose.position.x) < 0.1):
        return True
    return False

def rel_Goto (x, y, z):
    x = Drone_pose.pose.position.x + x
    y = Drone_pose.pose.position.y + y
    z = Drone_pose.pose.position.z + z
    set_position(x, y, z)

    rate.sleep()

    while not secure_accuracy(Drone_pose, goal_pose):
        set_position (x, y, z)

        rate.sleep()

def abs_Goto (x, y, z):
    set_position (x, y, z)

    rate.sleep()

    while not secure_accuracy(Drone_pose, goal_pose):
        set_position (x, y, z)

        rate.sleep()

print('criou funcao')

rospy.init_node('Vel_Control_Node', anonymous = True)

rate = rospy.Rate(20)

local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 100)

state_status_subscribe = rospy.Subscriber('/mavros/state', State, state)

arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

local_atual = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, drone_position)

print('sub e pubs')

for i in range(300):
    local_position_pub.publish(goal_pose)
    rate.sleep()

rospy.loginfo("[ROS] SETUP CONCLUIDO")

cont = 0


while not rospy.is_shutdown():
    if current_state != "OFFBOARD" or not current_state.armed:
        arm (True)
        set_mode(custom_mode = "OFFBOARD")


    print(str(current_state.mode))

    if current_state.armed == True:
        rospy.loginfo("DRONE ARMED")

    if current_state.mode == "OFFBOARD":
        rospy.loginfo('OFFBOARD mode setted')

    abs_Goto (0, 0, 2)
    abs_Goto (2, 0, 2)
    rel_Goto (2, 0, 0)
    abs_Goto (2, 0, 2)
