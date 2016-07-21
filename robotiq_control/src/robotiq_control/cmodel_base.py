#!/usr/bin/env python
from math import ceil
import time
import threading
from robotiq_msgs.msg import CModelStatus
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadInputRegistersResponse


class ComModbusTcp:
  def __init__(self):
    self.client = None
    self.lock = threading.Lock()
  
  def connectToDevice(self, address):
    """
    Connection to the client - the method takes the IP address (as a string, e.g. '192.168.1.11') as an argument.
    """
    self.client = ModbusTcpClient(address)

  def disconnectFromDevice(self):
    """Close connection"""
    self.client.close()

  def sendCommand(self, data):
    """
    Send a command to the Gripper - the method takes a list of uint8 as an argument. 
    The meaning of each variable depends on the Gripper model 
    """
    # make sure data has an even number of elements
    if(len(data) % 2 == 1):
      data.append(0)
    # Initiate message as an empty list
    message = []
    # Fill message by combining two bytes in one register
    for i in range(0, len(data)/2):
      message.append((data[2*i] << 8) + data[2*i+1])
    # TODO: Implement try/except
    with self.lock:
      self.client.write_registers(0, message)

  def getStatus(self, numBytes):
    """
    Sends a request to read, wait for the response and returns the Gripper status. 
    The method gets the number of bytes to read as an argument
    """
    numRegs = int(ceil(numBytes/2.0))
    # TODO: Implement try/except 
    # Get status from the device
    with self.lock:
      response = self.client.read_input_registers(0, numRegs)
    # Instantiate output as an empty list
    output = []
    # Fill the output with the bytes in the appropriate order
    for i in range(0, numRegs):
      output.append((response.getRegister(i) & 0xFF00) >> 8)
      output.append( response.getRegister(i) & 0x00FF)
    # Output the result
    return output


class RobotiqCModel:
  def __init__(self):
    #Initiate output message as an empty list
    self.message = []

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
    
  def refreshCommand(self, command):
    # Limit the value of each variable
    command = self.verifyCommand(command)
    # Initiate command as an empty list
    self.message = []
    # Build the command with each output variable
    # TODO: add verification that all variables are in their authorized range
    self.message.append(command.rACT + (command.rGTO << 3) + (command.rATR << 4))
    self.message.append(0)
    self.message.append(0)
    self.message.append(command.rPR)
    self.message.append(command.rSP)
    self.message.append(command.rFR)

  def sendCommand(self):
    self.client.sendCommand(self.message)

  def getStatus(self):
    """
    Request the status from the gripper and return it in the CModelStatus msg type.
    """
    #Acquire status from the Gripper
    status = self.client.getStatus(6);
    # Message to report the gripper status
    message = CModelStatus()
    #Assign the values to their respective variables
    message.gACT = (status[0] >> 0) & 0x01;
    message.gGTO = (status[0] >> 3) & 0x01;
    message.gSTA = (status[0] >> 4) & 0x03;
    message.gOBJ = (status[0] >> 6) & 0x03;
    message.gFLT =  status[2]
    message.gPR  =  status[3]
    message.gPO  =  status[4]
    message.gCU  =  status[5]
    return message
