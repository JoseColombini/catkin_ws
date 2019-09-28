#!/usr/bin/env python

import rospy
import smach

import mavros_msgs

from geometry_msgs import PoseStamped
from mavros_msgs import State
from sensor_msgs import NavSatFix
from mavros_msgs import GlobalPositionTarget
