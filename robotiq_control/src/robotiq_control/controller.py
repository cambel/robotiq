#!/usr/bin/env python
"""
This module contains Robotiq class as an interface to control Robotiq
gripper directly using ROS action client.
"""

import rospy
import actionlib
# Messages
from robotiq_msgs.msg import CModelCommandAction, CModelCommandGoal

def solve_namespace(namespace=''):
  """
  Appends neccessary slashes required for a proper ROS namespace.
  @type namespace: string
  @param namespace: namespace to be fixed.
  @rtype: string
  @return: Proper ROS namespace.
  """
  if len(namespace) == 0:
    namespace = rospy.get_namespace()
  elif len(namespace) == 1:
    if namespace != '/':
      namespace = '/' + namespace + '/'
  else:
    if namespace[0] != '/':
      namespace = '/' + namespace
    if namespace[-1] != '/':
      namespace += '/'
  return namespace

class Robotiq(object):
  """
  Interface class to control the Robotiq gripper using ROS action.
  """
  def __init__(self, namespace=''):
    self.ns = solve_namespace(namespace)

    action_server = self.ns + 'gripper/gripper_action_controller'
    self._client = actionlib.SimpleActionClient(action_server, CModelCommandAction)
    self._goal = CModelCommandGoal()
    rospy.loginfo("Waiting for [%s] action server" % action_server)
    server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
    if not server_up:
      rospy.logerr("Timed out waiting for Gripper Controller"
                   " Action Server to connect. Start the action server"
                   " before running this node.")
      rospy.signal_shutdown("Timed out waiting for Action Server")
      exit(1)
    rospy.loginfo("Successfully connected to [%s]" % action_server)

  def command(self, pos, vel, force):
    self._goal.position = pos
    self._goal.velocity = vel
    self._goal.force = force
    self._client.send_goal_and_wait(self._goal)

  def command_no_wait(self, pos, vel, force):
    self._goal.position = pos
    self._goal.velocity = vel
    self._goal.force = force
    self._client.send_goal(self._goal)

  def open(self, delay=0):
    rospy.sleep(delay)
    self.command(0.085, 0.1, 80)

  def close(self):
    self.command(0.0, 0.1, 80)
 
  def stop(self):
    self._client.cancel_goal()
 
  def wait(self, timeout=15.0):
    self._client.wait_for_result(timeout=rospy.Duration(timeout))
 
  def result(self):
    return self._client.get_result()

  def action_done(self):
    return self._client.simple_state == actionlib.SimpleGoalState.DONE
