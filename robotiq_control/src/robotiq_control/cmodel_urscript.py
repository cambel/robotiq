#!/usr/bin/env python
from robotiq_msgs.msg import CModelStatus, CModelCommand
import rospy
import std_msgs.msg

import os, rospkg

class RobotiqCModelURScript:
  def __init__(self, topic):
    #Initiate output message as an empty list
    self.status = CModelStatus()
    self.last_command = CModelCommand()
    self.pub = rospy.Publisher(topic, std_msgs.msg.String, queue_size=1)
    self.rospack = rospkg.RosPack()

    self.is_moving = False
    self.is_closing = False
    self.command_received_time = rospy.get_rostime()
    self.status_update_timer = rospy.Timer(rospy.Duration(.1), self.updateStatus)

    self.long_move = False # Hacky

  def verifyCommand(self, command):
    # Verify that each variable is in its correct range
    command.rACT = max(0, command.rACT)
    command.rACT = min(1, command.rACT)
    
    command.rGTO = max(0, command.rGTO)
    command.rGTO = min(1, command.rGTO)

    command.rATR = max(0, command.rATR)
    command.rATR = min(1, command.rATR)
    
    command.rPR  = max(0,   command.rPR)
    command.rPR  = min(255, command.rPR)
    
    command.rSP  = max(0,   command.rSP)
    command.rSP  = min(255, command.rSP)
    
    command.rFR  = max(0,   command.rFR)
    command.rFR  = min(255, command.rFR)
    
    # Return the cropped command
    return command
    

  def sendCommand(self, command):
    # Limit the value of each variable
    command = self.verifyCommand(command)
    
    if command.rPR == self.last_command.rPR and command.rACT == self.last_command.rACT and command.rFR == self.last_command.rFR and command.rSP == self.last_command.rSP:
      rospy.logdebug("Ignoring command that was sent again")
      return

    command_script = self.buildCommandProgram(command)
    self.pub.publish(command_script)
    self.last_command = command
    
    # Set status, assuming that the command succeeded
    # https://robotiq.com/support/2f-85-2f-140/downloads-instruction-manual Section 4
    self.status.gACT = command.rACT
    self.status.gGTO = command.rGTO
    self.status.gSTA = 3    # This pretends that the hand is online

    if command.rGTO == 1 and command.rPR < 10:  # Gripper opens
      self.status.gOBJ = 0   # Pretend that no object was grasped
      self.is_moving = True
      self.is_closing = False
    if command.rGTO == 1 and command.rPR > 200:
      self.is_moving = True
      self.is_closing = True
    self.command_received_time = rospy.get_rostime()

    self.status.gFLT = 0 # Fault
    self.status.gPR  = command.rPR
    # self.status.gPO  = command.rPR    # The "actual" position is set by updateStatus after a delay
    self.status.gCU  = 10 # Current

    ## Hacky
    if command.rGTO == 1 and command.rSP < 50:  # Gripper opens
      self.long_move = True
    return True

  def updateStatus(self, timerevent):
    if self.is_moving:
      time_since_command = rospy.get_rostime() - self.command_received_time
      # This duration should cover sending delay + UR parsing time + gripper mvt time
      if (not self.long_move and time_since_command > rospy.Duration(1.0)) or (self.long_move and time_since_command > rospy.Duration(4.0)):
      # if time_since_command > rospy.Duration(1.0):
      #   if self.long_move and time_since_command < rospy.Duration(4.0)):
      #     return
        if self.is_closing:
          self.status.gOBJ = 1
        self.is_moving = False
        self.is_closing = False
        self.long_move = False
        self.status.gPO = self.status.gPR
        rospy.logdebug("Update status to gPO = " + str(self.status.gPO))

  def getStatus(self):
    """
    Sends the current status of the gripper (estimated from the commands sent to the gripper, since there is no feedback as of yet).
    """
    return self.status

  def disconnectFromDevice(self):
    """Close connection"""

  def buildCommandProgram(self, message):
    """Constructs a program to send to the robot."""
    complete_program = ""
    
    # Construct the program containing the gripper URCap definitions
    program_template = open(os.path.join(self.rospack.get_path("robotiq_control"), "src", "robotiq_urscript.script"), 'rb')
    program_line = program_template.read(1024)
    while program_line:
        #complete_program += program_line.decode()
        complete_program += program_line
        program_line = program_template.read(1024)
    
    # Add the parts with the commands we received. 
    # Setting force/speed every time is a bit wasteful, but it is good enough for now.
    complete_program += "rq_set_force("+str(message.rFR)+")\n"
    complete_program += "rq_set_speed("+str(message.rSP)+")\n"
    complete_program += "rq_move("+str(message.rPR)+")\n"
    complete_program += "end"

    rospy.loginfo("Sending command to go to pos " + str(message.rPR) + " with force " + str(message.rFR) + " and speed " + str(message.rSP))

    # Wrap as std_msgs/String
    program_msg = std_msgs.msg.String()
    program_msg.data = complete_program

    return program_msg
