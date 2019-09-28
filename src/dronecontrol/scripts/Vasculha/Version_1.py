#!/usr/bin/env python

import Fmove

rospy.init_node('Vel_Control_Node', anonymous = True)

rate = rospy.Rate(20)

for i in range(300):
    local_position_pub.publish(goal_pose)
    rate.sleep()

rospy.loginfo("[ROS] SETUP CONCLUIDO")

while not rospy.is_shutdown():
    abs_Goto(0, 0, 2)
    abs_spd_Goto(10, 0, 2)
    rel_spd_Goto(-5, 0, 0)
