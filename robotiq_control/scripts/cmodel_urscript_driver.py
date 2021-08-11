#!/usr/bin/env python
import os
import sys
import socket
import rospy
from robotiq_control.cmodel_urscript import RobotiqCModelURScript
from robotiq_msgs.msg import CModelCommand, CModelStatus

def mainLoop(urscript_topic):
  # Gripper is a C-Model that is connected to a UR controller. 
  # Commands should be published to ur_modern_driver's URScript topic.
  gripper = RobotiqCModelURScript(urscript_topic)
  # The Gripper status
  pub = rospy.Publisher('status', CModelStatus, queue_size=3)
  # The Gripper command
  rospy.Subscriber('command', CModelCommand, gripper.sendCommand)
  
  while not rospy.is_shutdown():
    # Get and publish the Gripper status
    status = gripper.getStatus()
    pub.publish(status)
    # Wait a little
    rospy.sleep(0.03)

if __name__ == '__main__':
  rospy.init_node('cmodel_urscript_driver')
  try:
    mainLoop(sys.argv[1])
  except rospy.ROSInterruptException: pass
