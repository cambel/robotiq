#!/usr/bin/env python
import os
import sys
import socket
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
  # Verify user gave a legal IP address
  try:
    ip = sys.argv[1]
    socket.inet_aton(ip)
  except socket.error:
    rospy.logfatal('[cmodel_tcp_driver] Please provide a valid IP address')
  # Run the main loop
  try:
    mainLoop(sys.argv[1])
  except rospy.ROSInterruptException: pass
