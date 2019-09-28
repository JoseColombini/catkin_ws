#!/usr/bin/env python

import rospy
import mavros_msgs
from mavros_msgs import srv
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

import math

#Global constats
accuracy = 0.1
spd = 0.5
drone_ideal = [0, 0, 0]

#Variaveis
goal_pose = PoseStamped()
current_state = State()
local = PoseStamped()
Drone_pose = PoseStamped()
Drone_speed = TwistStamped()

#State of drone
def state(state_data):
    global current_state
    current_state = state_data


#define the secure_accuracy to proced the mission
def secure_accuracy (x,y):
    global accuracy
    if (abs(x.pose.position.z - y.pose.position.z) < accuracy) and (abs(x.pose.position.y - y.pose.position.y) < accuracy) and (abs(x.pose.position.x - y.pose.position.x) < accuracy):
        return True
    return False


#Moviment func

#set position drone is going to (base for others func)
def set_position(x, y, z):
    global goal_pose
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z
    local_position_pub.publish(goal_pose)

def set_speed(x, y, z):
    global Drone_speed

    Drone_speed.twist.linear.x = x
    Drone_speed.twist.linear.y = y
    Drone_speed.twist.linear.z = z
    spd_drone_pub.publish(Drone_speed)
    rate.sleep()

#identify the drone position (used in relative moviment)
def drone_position(local):
    global Drone_pose

    Drone_pose.pose.position.x = local.pose.position.x
    Drone_pose.pose.position.y = local.pose.position.y
    Drone_pose.pose.position.z = local.pose.position.z

#relavtive moviment for drone
def rel_Goto (x, y, z):
    global Drone_pose

    x = Drone_pose.pose.position.x + x
    y = Drone_pose.pose.position.y + y
    z = Drone_pose.pose.position.z + z
    set_position(x, y, z)

    rate.sleep()

    while not secure_accuracy(Drone_pose, goal_pose):
        set_position (x, y, z)

        rate.sleep()


#set the position to move the drone base on the takeoff point
def abs_Goto (x, y, z):
    set_position (x, y, z)
    rate.sleep()

    while not secure_accuracy(Drone_pose, goal_pose):
        set_position (x, y, z)

        rate.sleep()

#control drone move by Drone_speed
def abs_spd_Goto (x, y, z):
    global Drone_pose
    global goal_pose
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z

    vx = 0
    vy = 0
    vz = 0
    print ("pre while")
    while not secure_accuracy(Drone_pose, goal_pose):
        print ("passo 1")
        if (abs(x - Drone_pose.pose.position.x) > accuracy):
            print("aqui era para entrar")
            vx =  math.copysign(spd, x - Drone_pose.pose.position.x)
        if (abs(y - Drone_pose.pose.position.y) > accuracy):
            vy =  math.copysign(spd, y - Drone_pose.pose.position.y)
        if (abs(z - Drone_pose.pose.position.z) > accuracy):
            vz =  math.copysign(spd, z - Drone_pose.pose.position.z)
        set_speed(vx, vy, vz)
        rate.sleep()
    abs_Goto(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z)

def rel_spd_Goto (x, y, z):
    global Drone_pose
    global goal_pose
    goal_pose.pose.position.x = Drone_pose.pose.position.x + x
    goal_pose.pose.position.y = y + Drone_pose.pose.position.y
    goal_pose.pose.position.z = z + Drone_pose.pose.position.z

    vx = 0
    vy = 0
    vz = 0

    while not secure_accuracy(Drone_pose, goal_pose):
        if x:
            vx =  math.copysign(spd, x)
        if y:
            vy =  math.copysign(spd, y)
        if z:
            vz =  math.copysign(spd, z)
        set_speed(vx, vy, vz)
        rate.sleep()
    abs_Goto(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z)



#define the publish and subs for funcs above
local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 100)

spd_drone_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 100)

state_status_subscribe = rospy.Subscriber('/mavros/state', State, state)

arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

local_atual = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, drone_position)


#velocity_set

rospy.init_node('Fmove', anonymous = True)

rate = rospy.Rate(20)

for i in range(300):
    local_position_pub.publish(goal_pose)
    rate.sleep()

rospy.loginfo("[ROS] SETUP CONCLUIDO")

while not rospy.is_shutdown():
    if current_state != "OFFBOARD" or not current_state.armed:
        arm (True)
        set_mode(custom_mode = "OFFBOARD")


    abs_Goto(0, 0, 2)
#    rel_spd_Goto (1, 0, 0)
    abs_spd_Goto(1, 3, 2)
#    rel_Goto(-2, 0, 0)
#    while True:
#        abs_Goto(0, 0, 2)
