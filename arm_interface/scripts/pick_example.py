#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from arm_interface.srv import pick, place
from geometry_msgs.msg import Pose

# Pick object at (x, y, z) wrt the base of the arm
def pickme(x,y,z):
    rospy.wait_for_service('/arm_interface/pick')
    try:
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        pickfn = rospy.ServiceProxy('/arm_interface/pick', pick)
        
        resp1 = pickfn(target_pose)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Pick object at (x, y, z) wrt the base of the arm
def placeme(x, y, z):
    rospy.wait_for_service('/arm_interface/place')
    try:
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        placefn = rospy.ServiceProxy('/arm_interface/place', place)
    
        resp1 = placefn(target_pose)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

        
def usage():
    return "%s [pick,place] x y z"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        action = sys.argv[1]
        x = float(sys.argv[2])
        y = float(sys.argv[3])
        z = float(sys.argv[4])
    else:
        print(usage())
        sys.exit(1)
    if action not in ['pick', 'place']:
        print(usage())
        sys.exit(1)
    print("Requesting to %s (%s, %s, %s)"%(action, x, y, z))
    if action == 'pick':
        pickme(x, y, z)
    else:
        placeme(x, y, z)
