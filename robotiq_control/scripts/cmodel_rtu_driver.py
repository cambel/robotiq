#!/usr/bin/env python
import os
import sys
import socket
import rospy
from robotiq_control.cmodel_base import RobotiqCModel, ComModbusRtu
from robotiq_msgs.msg import CModelCommand, CModelStatus

def mainLoop(device):
  # Gripper is a C-Model with a TCP connection
  gripper = RobotiqCModel()
  gripper.client = ComModbusRtu()
  # We connect to the device name received as an argument
  rospy.loginfo("Connecting to device " + str(device))
  gripper.client.connectToDevice(device)
  # The Gripper status
  pub = rospy.Publisher('status', CModelStatus, queue_size=3)
  # The Gripper command
  rospy.Subscriber('command', CModelCommand, gripper.refreshCommand)
  
  while not rospy.is_shutdown():
    # Get and publish the Gripper status
    status = gripper.getStatus()
    pub.publish(status)
    # Wait a little
    rospy.sleep(0.03)
    # Send the most recent command
    gripper.sendCommand()
    # Wait a little
    rospy.sleep(0.03)

if __name__ == '__main__':
  rospy.init_node('cmodel_rtu_driver')
  # Run the main loop
  try:
    mainLoop(sys.argv[1])
  except rospy.ROSInterruptException: pass
