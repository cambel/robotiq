#! /usr/bin/env python
import os
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from robotiq_msgs.msg import CModelCommand, CModelStatus


class ControllerEmulator(object):
  def __init__(self):
    expected_controller = 'gazebo_gripper'
    self.position = None
    # Setup publishers and subscribers
    ns = rospy.get_namespace()
    status_pub = rospy.Publisher('gripper/status', CModelStatus, queue_size=1)
    gazebo_pub = rospy.Publisher('{0}/command'.format(expected_controller), Float64, queue_size=1)
    rospy.Subscriber('gripper/command', CModelCommand, self.cb_gripper_command, queue_size=1)
    rospy.Subscriber('joint_states', JointState, self.cb_joint_states, queue_size=1)
    # Check that the gazebo_gripper controller exists
    #~ controller_list_srv = ns + 'controller_manager/list_controllers'
    controller_list_srv = 'controller_manager/list_controllers'
    rospy.loginfo('Waiting for the {0} controller'.format(expected_controller))
    rospy.wait_for_service(controller_list_srv, timeout=30.0)
    list_controllers = rospy.ServiceProxy(controller_list_srv, ListControllers)
    found = False
    while not rospy.is_shutdown() and not found:
      try:
        res = list_controllers()
        for state in res.controller:
          if state.name == expected_controller:
            found = True
            break
      except: pass
      rospy.sleep(0.01)
    while not rospy.is_shutdown() and self.position is None:
      rospy.sleep(0.01)
    self.jnt_command = self.position
    rospy.loginfo('[cmodel_controller_emulator] successfully initialized')
    # Report that the gripper is ready to receive commands
    status = CModelStatus()
    status.gACT = 1
    status.gSTA = 3
    while not rospy.is_shutdown():
      gPO = int(round(230*self.position/0.8))
      status.gPO = min(max(0, gPO), 255)
      status_pub.publish(status)
      rospy.sleep(0.05)
      gazebo_pub.publish(self.jnt_command)
      rospy.sleep(0.05)
  
  def cb_gripper_command(self, msg):
    self.jnt_command = 0.8*msg.rPR/230. 
  
  def cb_joint_states(self, msg):
    for joint_name in msg.name:
      if joint_name == 'robotiq_85_left_knuckle_joint':
        idx = msg.name.index(joint_name)
        self.position = msg.position[idx]
        break


if __name__ == "__main__":
  # Initialize the node
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  emulator = ControllerEmulator()


