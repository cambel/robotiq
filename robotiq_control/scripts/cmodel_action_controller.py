#!/usr/bin/env python
import rospy
import os
import numpy as np
from sensor_msgs.msg import JointState
# Actionlib
from actionlib import SimpleActionServer
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult, GripperCommandFeedback
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
        self.last_position = 0.0
        self.last_time = rospy.get_time()

        self._ns = rospy.get_namespace()
        # Read configuration parameters
        self._fb_rate = read_parameter(self._ns + 'gripper_action_controller/publish_rate', 60.0)
        self._min_gap_counts = read_parameter(self._ns + 'gripper_action_controller/min_gap_counts', 230.)
        self._counts_to_meters = read_parameter(self._ns + 'gripper_action_controller/counts_to_meters', 0.8)
        self._min_gap = read_parameter(self._ns + 'gripper_action_controller/min_gap', 0.0)
        self._max_gap = read_parameter(self._ns + 'gripper_action_controller/max_gap', 0.085)
        self._min_speed = read_parameter(self._ns + 'gripper_action_controller/min_speed', 0.013)
        self._max_speed = read_parameter(self._ns + 'gripper_action_controller/max_speed', 0.1)
        self._min_force = read_parameter(self._ns + 'gripper_action_controller/min_force', 40.0)
        self._max_force = read_parameter(self._ns + 'gripper_action_controller/max_force', 100.0)
        self._joint_name = read_parameter(self._ns + 'gripper_action_controller/joint_name', 'robotiq_85_left_knuckle_joint')
        self._gripper_prefix = read_parameter(self._ns + 'gripper_prefix', "")   # Used for updating joint state
        # Configure and start the action server
        self._status = CModelStatus()
        self._name = self._ns + 'gripper_action_controller'
        self._server = SimpleActionServer(self._name, CModelCommandAction, execute_cb=self._execute_no_delay_cb, auto_start=False)
        self._server_old = SimpleActionServer(self._name + '_old', CModelCommandAction, execute_cb=self._execute_cb, auto_start=False)
        self._simple_gripper_server = SimpleActionServer(self._ns + 'simple_gripper_action_controller', GripperCommandAction, execute_cb=self._simple_gripper_action_cb, auto_start=False)

        self.status_pub = rospy.Publisher('gripper_status', CModelCommandFeedback, queue_size=1)
        self.js_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.Subscriber('status', CModelStatus, self._status_cb, queue_size=1)
        self._cmd_pub = rospy.Publisher('command', CModelCommand, queue_size=1)
        working = True
        rospy.sleep(1.0)   # Wait before checking status with self._ready()
        if activate and not self._ready():
            rospy.sleep(2.0)
            working = self._activate()
        if not working:
            return
        self._server.start()
        self._server_old.start()
        self._simple_gripper_server.start()
        rospy.logdebug('%s: Started' % self._name)

    def _preempt(self):
        # self._stop()
        # rospy.loginfo('%s: Preempted' % self._name)
        self._server.set_preempted()

    def _status_cb(self, msg):
        self.current_time = rospy.get_time()
        dt = self.current_time - self.last_time
        # self.current_position = self._get_position()  # opening width
        self.current_position = self._counts_to_meters * self._status.gPO / self._min_gap_counts  # angle
        self.current_velocity = (self.current_position - self.last_position) / (dt + 1e-8)

        self._status = msg
        # Publish the joint_states for the gripper
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.name.append(self._gripper_prefix + self._joint_name)
        js_msg.position.append(self.current_position)
        js_msg.velocity.append(self.current_velocity)
        js_msg.effort.append(0.0)
        self.js_pub.publish(js_msg)

        self.last_position = self.current_position
        self.last_time = self.current_time

        # Publish the gripper status (to easily access gripper width)
        feedback = CModelCommandFeedback()
        feedback.activated = self._ready()
        feedback.position = self._get_position()
        # feedback.velocity = self.current_velocity
        feedback.stalled = self._stalled()
        # # feedback.reached_goal = self._reached_goal(position)
        try:
            self.status_pub.publish(feedback)
        except rospy.ROSException:
            pass

    def _simple_gripper_action_cb(self, goal: GripperCommandGoal):
        # Check that the gripper is active. If not, activate it.
        if not self._ready():
            if not self._silent_activate():
                self._simple_gripper_server.set_preempted()
                return

        # check that preempt has not been requested by the client
        if self._simple_gripper_server.is_preempt_requested():
            self._simple_gripper_server.set_preempted()
            return

        # compute target opening width
        target_gPO = goal.command.position * self._min_gap_counts / self._counts_to_meters
        pos = np.clip((self._max_gap - self._min_gap)/(-self._min_gap_counts)*(target_gPO-self._min_gap_counts), self._min_gap, self._max_gap)

        # Clip the goal
        position = np.clip(pos, self._min_gap, self._max_gap)
        velocity = self._min_speed  # TODO: Fix hard-coded params
        force = self._max_force  # TODO: Fix hard-coded params

        # Send the goal to the gripper and feedback to the action client
        self._status.gOBJ = 0  # R.Hanai

        feedback = GripperCommandFeedback()
        rate = rospy.Rate(self._fb_rate)

        command_sent_time = rospy.get_rostime()
        while not self._reached_goal(position):
            self._goto_position(position, velocity, force)
            if rospy.is_shutdown() or self._simple_gripper_server.is_preempt_requested():
                self._simple_gripper_server.set_preempted()
                return
            feedback.position = self._get_position()
            feedback.stalled = self._stalled()
            feedback.reached_goal = self._reached_goal(position)
            self._simple_gripper_server.publish_feedback(feedback)
            rate.sleep()

            time_since_command = rospy.get_rostime() - command_sent_time
            if time_since_command > rospy.Duration(1.0) and self._stalled():
                break

        result = GripperCommandResult()
        result.position = self._get_position()
        result.stalled = self._stalled()
        result.reached_goal = self._reached_goal(position)
        self._simple_gripper_server.set_succeeded(result)

    def _execute_no_delay_cb(self, goal):
        # Check that the gripper is active. If not, activate it.
        if not self._ready():
            if not self._silent_activate():
                self._server.set_preempted()
                return

        # check that preempt has not been requested by the client
        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        # Clip the goal
        position = np.clip(goal.position, self._min_gap, self._max_gap)
        velocity = np.clip(goal.velocity, self._min_speed, self._max_speed)
        force = np.clip(goal.force, self._min_force, self._max_force)

        # Send the goal to the gripper and feedback to the action client
        self._status.gOBJ = 0  # R.Hanai
        self._goto_position(position, velocity, force)

        result = CModelCommandResult()
        result.position = self._get_position()
        result.stalled = self._stalled()
        result.reached_goal = self._reached_goal(position)
        self._server.set_succeeded(result)

    def _execute_cb(self, goal):
        # Check that the gripper is active. If not, activate it.
        if not self._ready():
            if not self._activate():
                rospy.logwarn('%s could not accept goal because the gripper is not yet active' % self._name)
                return
        # check that preempt has not been requested by the client
        if self._server_old.is_preempt_requested():
            self._server_old.set_preempted()
            return
        # Clip the goal
        position = np.clip(goal.position, self._min_gap, self._max_gap)
        velocity = np.clip(goal.velocity, self._min_speed, self._max_speed)
        force = np.clip(goal.force, self._min_force, self._max_force)
        # Send the goal to the gripper and feedback to the action client
        rate = rospy.Rate(self._fb_rate)
        rospy.logdebug('%s: Moving gripper to position: %.3f ' % (self._name, position))

        self._status.gOBJ = 0  # R.Hanai

        feedback = CModelCommandFeedback()

        command_sent_time = rospy.get_rostime()
        while not self._reached_goal(position):
            self._goto_position(position, velocity, force)
            if rospy.is_shutdown() or self._server_old.is_preempt_requested():
                self._server_old.set_preempted()
                return
            feedback.position = self._get_position()
            feedback.stalled = self._stalled()
            feedback.reached_goal = self._reached_goal(position)
            self._server_old.publish_feedback(feedback)
            rate.sleep()

            time_since_command = rospy.get_rostime() - command_sent_time
            if time_since_command > rospy.Duration(0.5) and self._stalled():
                break

        rospy.logdebug('%s: Succeeded' % self._name)
        result = CModelCommandResult()
        result.position = self._get_position()
        result.stalled = self._stalled()
        result.reached_goal = self._reached_goal(position)
        self._server_old.set_succeeded(result)

    def _silent_activate(self):
        command = CModelCommand()
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 150
        self._cmd_pub.publish(command)

    def _activate(self, timeout=5.0):
        command = CModelCommand()
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 150
        start_time = rospy.get_time()
        while not self._ready():
            if rospy.is_shutdown():
                self._preempt()
                return False
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn('Failed to activate gripper in ns [%s]' % (self._ns))
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

    def _reached_goal(self, goal, tol=0.003):
        # rospy.loginfo('REACHED_GOAL: goal=%f, current=%f'%(goal, self._get_position()))
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
    rospy.loginfo('Shutting down [%s] node' % node_name)
