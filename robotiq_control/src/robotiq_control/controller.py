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
  Interface class to control the Robotiq gripper using ROS action client.
  It connects to the C{gripper/gripper_action_controller} action server.
  """
  def __init__(self, namespace=''):
    """
    Robotiq constructor. It is connected to the 
    C{gripper/gripper_action_controller} action server.
    @type namespace: string
    @param namespace: Override ROS namespace manually. Useful when accessing multiple Robotiq grippers from the same node.
    """
    self.ns = solve_namespace(namespace)

    action_server = self.ns + 'gripper/gripper_action_controller'
    self._client = actionlib.SimpleActionClient(action_server, CModelCommandAction)
    self._goal = CModelCommandGoal()
    rospy.logdebug('Waiting for [%s] action server' % action_server)
    server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
    if not server_up:
      rospy.logerr('Timed out waiting for Gripper Controller'
                   ' Action Server to connect. Start the action server'
                   ' before running this node.')
      rospy.signal_shutdown("Timed out waiting for Action Server")
      exit(1)
    rospy.loginfo('Robotiq successfully initialized. ns: {0}'.format(namespace))

  def command(self, pos, vel, force, wait=True):
    """
    Send commond to the connected gripper.
    @type  pos: float
    @param pos: Desired position.
    @type  vel: float
    @param vel: Desired velocity.
    @type  force: float
    @param force: Desired grasping force.
    """
    self._goal.position = pos
    self._goal.velocity = vel
    self._goal.force = force
    if wait:
      self._client.send_goal_and_wait(self._goal)
    else:
      self._client.send_goal(self._goal)

  def open(self, delay=0):
    """
    Open the connected gripper.
    @type  delay: float
    @param delay: Delay time before execution. Useful when operator 
                  is away from the gripper.
    """
    rospy.sleep(delay)
    self.command(0.085, 0.1, 80)

  def close(self):
    """
    Close the connected gripper.
    """
    self.command(0.0, 0.1, 80)
 
  def stop(self):
    """
    Cancel current goal.
    """
    self._client.cancel_goal()
 
  def wait(self, timeout=15.0):
    """
    Wait until current goal motion is finished.
    @type  timeout: float
    @param timeout: Maximum wait time.
    """
    self._client.wait_for_result(timeout=rospy.Duration(timeout))
 
  def result(self):
    """
    Return action execution result.
    """
    return self._client.get_result()

  def is_action_done(self):
    """
    Return whether current action is finished.
    @rtype: bool
    @return: B{True} is current action is finished.
    """
    return self._client.simple_state == actionlib.SimpleGoalState.DONE
