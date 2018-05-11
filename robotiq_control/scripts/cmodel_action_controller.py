#!/usr/bin/env python
import rospy, os
import numpy as np
from sensor_msgs.msg import JointState
# Actionlib
from actionlib import SimpleActionServer
from robotiq_msgs.msg import (
    CModelCommand,
    CModelStatus,
    CModelCommandAction,
    CModelCommandFeedback,
    CModelCommandResult,
)

def read_parameter(name, default):
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)


class CModelActionController(object):
  def __init__(self, activate=True):
    self._ns = rospy.get_namespace()
    # Read configuration parameters
    self._fb_rate = read_parameter(self._ns + 'gripper_action_controller/publish_rate', 60.0)
    self._min_gap = read_parameter(self._ns + 'gripper_action_controller/min_gap', 0.0)
    self._min_gap_counts = read_parameter(self._ns + 'gripper_action_controller/min_gap_counts', 230.)
    self._max_gap = read_parameter(self._ns + 'gripper_action_controller/max_gap', 0.085)
    self._min_speed = read_parameter(self._ns + 'gripper_action_controller/min_speed', 0.013)
    self._max_speed = read_parameter(self._ns + 'gripper_action_controller/max_speed', 0.1)
    self._min_force = read_parameter(self._ns + 'gripper_action_controller/min_force', 40.0)
    self._max_force = read_parameter(self._ns + 'gripper_action_controller/max_force', 100.0)
    # Configure and start the action server
    self._status = CModelStatus()
    self._name = self._ns + 'gripper_action_controller'
    self._server = SimpleActionServer(self._name, CModelCommandAction, execute_cb=self._execute_cb, auto_start = False)
    self.js_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.Subscriber('status', CModelStatus, self._status_cb, queue_size=1)
    self._cmd_pub = rospy.Publisher('command', CModelCommand, queue_size=1)
    working = True
    if activate and not self._ready():
      rospy.sleep(2.0)
      working = self._activate()
    if not working:
      return
    self._server.start()
    rospy.logdebug('%s: Started' % self._name)

  def _preempt(self):
    self._stop()
    rospy.loginfo('%s: Preempted' % self._name)
    self._server.set_preempted()

  def _status_cb(self, msg):
    self._status = msg
    # Publish the joint_states for the gripper
    js_msg = JointState()
    js_msg.header.stamp = rospy.Time.now()
    js_msg.name.append('robotiq_85_left_knuckle_joint')
    js_msg.position.append(0.8*self._status.gPO/self._min_gap_counts)
    self.js_pub.publish(js_msg)

  def _execute_cb(self, goal):
    success = True
    # Check that the gripper is active. If not, activate it.
    if not self._ready():
      if not self._activate():
        rospy.logwarn('%s could not accept goal because the gripper is not yet active' % self._name)
        return
    # check that preempt has not been requested by the client
    if self._server.is_preempt_requested():
      self._preempt()
      return
    # Clip the goal
    position = np.clip(goal.position, self._min_gap, self._max_gap)
    velocity = np.clip(goal.velocity, self._min_speed, self._max_speed)
    force = np.clip(goal.force, self._min_force, self._max_force)
    # Send the goal to the gripper and feedback to the action client
    rate = rospy.Rate(self._fb_rate)
    rospy.logdebug('%s: Moving gripper to position: %.3f ' % (self._name, position))
    feedback = CModelCommandFeedback()
    while not self._reached_goal(position):
      self._goto_position(position, velocity, force)
      if rospy.is_shutdown() or self._server.is_preempt_requested():
        self._preempt()
        return
      feedback.position = self._get_position()
      feedback.stalled = self._stalled()
      feedback.reached_goal = self._reached_goal(position)
      self._server.publish_feedback(feedback)
      rate.sleep()
      if self._stalled():
        break
    rospy.logdebug('%s: Succeeded' % self._name)
    result = CModelCommandResult()
    result.position = self._get_position()
    result.stalled = self._stalled()
    result.reached_goal = self._reached_goal(position)
    self._server.set_succeeded(result)

  def _activate(self, timeout=5.0):
    command = CModelCommand();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
    start_time = rospy.get_time()
    while not self._ready():
      if rospy.is_shutdown():
        self._preempt()
        return False
      if rospy.get_time() - start_time > timeout:
        rospy.logwarn('Failed to activated gripper in ns [%s]' % (self._ns))
        return False
      self._cmd_pub.publish(command)
      rospy.sleep(0.1)
    rospy.loginfo('Successfully activated gripper in ns [%s]' % (self._ns))
    return True

  def _get_position(self):
    gPO = self._status.gPO
    pos = np.clip((self._max_gap - self._min_gap)/(-self._min_gap_counts)*(gPO-self._min_gap_counts), self._min_gap, self._max_gap)
    return pos

  def _goto_position(self, pos, vel, force):
    """
    Goto position with desired force and velocity
    @type  pos: float
    @param pos: Gripper width in meters
    @type  vel: float
    @param vel: Gripper speed in m/s
    @type  force: float
    @param force: Gripper force in N
    """
    command = CModelCommand()
    command.rACT = 1
    command.rGTO = 1
    command.rPR = int(np.clip((-self._min_gap_counts)/(self._max_gap - self._min_gap) * (pos - self._min_gap) + self._min_gap_counts, 0, self._min_gap_counts))
    command.rSP = int(np.clip((255)/(self._max_speed - self._min_speed) * (vel - self._min_speed), 0, 255))
    command.rFR = int(np.clip((255)/(self._max_force - self._min_force) * (force - self._min_force), 0, 255))
    self._cmd_pub.publish(command)

  def _moving(self):
    return self._status.gGTO == 1 and self._status.gOBJ == 0

  def _reached_goal(self, goal, tol = 0.003):
    return (abs(goal - self._get_position()) < tol)

  def _ready(self):
    return self._status.gSTA == 3 and self._status.gACT == 1

  def _stalled(self):
    return self._status.gOBJ == 1 or self._status.gOBJ == 2

  def _stop(self):
    command = CModelCommand()
    command.rACT = 1
    command.rGTO = 0
    self._cmd_pub.publish(command)
    rospy.logdebug('Stopping gripper in ns [%s]' % (self._ns))


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  cmodel_server = CModelActionController()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
