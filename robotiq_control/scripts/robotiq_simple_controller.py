#!/usr/bin/env python
import rospy
from robotiq_msgs.msg import CModelCommand
from time import sleep

def genCommand(char, command):
  """Update the command according to the character entered by the user."""    
  if char == 'a':
    command = CModelCommand();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
  if char == 'r':
    command = CModelCommand();
    command.rACT = 0
  if char == 'c':
    command.rPR = 255
  if char == 'o':
    command.rPR = 0
  #If the command entered is a int, assign this value to rPRA
  try: 
    command.rPR = int(char)
    if command.rPR > 255:
      command.rPR = 255
    if command.rPR < 0:
      command.rPR = 0
  except ValueError:
    pass
  if char == 'f':
    command.rSP += 25
    if command.rSP > 255:
      command.rSP = 255
  if char == 'l':
    command.rSP -= 25
    if command.rSP < 0:
      command.rSP = 0
  if char == 'i':
    command.rFR += 25
    if command.rFR > 255:
      command.rFR = 255
  if char == 'd':
    command.rFR -= 25
    if command.rFR < 0:
      command.rFR = 0
  return command

def askForCommand(command):
  """
  Ask the user for a command to send to the gripper.
  """
  currentCommand  = 'Simple C-Model Controller\n-----\nCurrent command:'
  currentCommand += '  rACT = '  + str(command.rACT)
  currentCommand += ', rGTO = '  + str(command.rGTO)
  currentCommand += ', rATR = '  + str(command.rATR)
  currentCommand += ', rPR = '   + str(command.rPR )
  currentCommand += ', rSP = '   + str(command.rSP )
  currentCommand += ', rFR = '   + str(command.rFR )
  print currentCommand
  strAskForCommand  = '-----\nAvailable commands\n\n'
  strAskForCommand += 'r: Reset\n'
  strAskForCommand += 'a: Activate\n'
  strAskForCommand += 'c: Close\n'
  strAskForCommand += 'o: Open\n'
  strAskForCommand += '(0-255): Go to that position\n'
  strAskForCommand += 'f: Faster\n'
  strAskForCommand += 'l: Slower\n'
  strAskForCommand += 'i: Increase force\n'
  strAskForCommand += 'd: Decrease force\n'
  strAskForCommand += '-->'
  return raw_input(strAskForCommand)

def publisher():
  """
  Main loop which requests new commands and publish them on the CModelRobotOutput topic.
  """
  rospy.init_node('robotiq_simple_controller')
  pub = rospy.Publisher('command', CModelCommand, queue_size=3)
  command = CModelCommand()
  while not rospy.is_shutdown():
    command = genCommand(askForCommand(command), command)
    pub.publish(command)
    rospy.sleep(0.1)

if __name__ == '__main__':
  publisher()
