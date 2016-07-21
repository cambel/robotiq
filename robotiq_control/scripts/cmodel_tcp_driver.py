#!/usr/bin/env python
import os, sys
import rospy
from robotiq_control.cmodel_base import RobotiqCModel, ComModbusTcp
from robotiq_msgs.msg import CModelCommand, CModelStatus

def mainLoop(address):
  # Gripper is a C-Model with a TCP connection
  gripper = RobotiqCModel()
  gripper.client = ComModbusTcp()
  # We connect to the address received as an argument
  gripper.client.connectToDevice(address)
  # The Gripper status
  pub = rospy.Publisher('status', CModelStatus, queue_size=3)
  # The Gripper command
  rospy.Subscriber('command', CModelCommand, gripper.refreshCommand)
  
  while not rospy.is_shutdown():
    # Get and publish the Gripper status
    status = gripper.getStatus()
    pub.publish(status)
    # Wait a little
    rospy.sleep(0.05)
    # Send the most recent command
    gripper.sendCommand()
    # Wait a little
    rospy.sleep(0.05)

if __name__ == '__main__':
  rospy.init_node('cmodel_tcp_driver')
  try:
    # TODO: Add verification that the argument is an IP address
    mainLoop(sys.argv[1])
  except rospy.ROSInterruptException: pass
