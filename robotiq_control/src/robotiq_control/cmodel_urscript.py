#!/usr/bin/env python
from robotiq_msgs.msg import CModelStatus
import rospy
import std_msgs.msg

import os, rospkg

class RobotiqCModelURScript:
  def __init__(self, topic):
    #Initiate output message as an empty list
    self.status = CModelStatus()
    self.pub = rospy.Publisher(topic, std_msgs.msg.String, queue_size=1)
    self.rospack = rospkg.RosPack()

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
    
    command_script = self.buildCommandProgram(command)
    self.pub.publish(command_script)
    rospy.loginfo("Sleeping for .5 seconds for the gripper")
    rospy.sleep(.5)
    
    # Set status, assuming that the command succeeded
    self.status.gACT = command.rACT
    # self.status.gGTO = command.rGTO
    # self.status.gSTA = command.rSTA
    # self.status.gOBJ = command.rOBJ
    # self.status.gFLT = command.rFLT
    self.status.gPR  = command.rPR 
    # self.status.gPO  = command.rPO 
    # self.status.gCU  = command.rCU 
    return True

  def getStatus(self):
    """
    Sends the current status of the gripper (estimated from the commands sent to the gripper, since there is no feedback as of yet).
    """
    return self.status

  def disconnectFromDevice(self):
    """Close connection"""
    # self.client.close()


  def buildCommandProgram(self, message):
    """Constructs a program to send to the robot."""
    complete_program = ""
    
    # Construct the program containing the gripper URCap definitions
    program_template = open(os.path.join(self.rospack.get_path("robotiq_control"), "src", "robotiq_urscript_move.ur"), 'rb')
    program_line = program_template.read(1024)
    while program_line:
        #complete_program += program_line.decode()
        complete_program += program_line
        program_line = program_template.read(1024)
    
    # Add the parts with the commands we received
    # TODO: Allow force + speed setting here instead of just setting the position
    complete_program += "rq_move("+str(message.rPR)+")\n"
    complete_program += "end"

    # Wrap as std_msgs/String
    program_msg = std_msgs.msg.String()
    program_msg.data = complete_program

    return program_msg
