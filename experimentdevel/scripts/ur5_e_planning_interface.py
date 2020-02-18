#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman
# Modified: Jason Dekarske (2/17/2020)

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import rosplan_dispatch_msgs.msg  
import rosplan_knowledge_msgs.srv
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_e_planning_interface', anonymous=True)

## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
## kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
## for getting, setting, and updating the robot's internal understanding of the
## surrounding world:
scene = moveit_commander.PlanningSceneInterface()

## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
## to a planning group (group of joints).  In this tutorial the group is the primary
## arm joints in the Panda robot, so we set the group's name to "panda_arm".
## If you are using a different robot, change this value to the name of your robot
## arm planning group.
## This interface can be used to plan and execute motions:
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

action_feedback_pub = rospy.Publisher('/rosplan_plan_dispatcher/action_feedback', rosplan_dispatch_msgs.msg.ActionFeedback, queue_size=10)

# rospy.wait_for_service('/rosplan_knowledge_base/status/update')
# update_kb = rospy.ServiceProxy('/rosplan_knowledge_base/status/update', rosplan_knowledge_msgs.srv.KnowledgeUpdateService)

planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

def go_to_pose_goal(wp):
  x = 0.400
  y = 0.400
  z = 0.400
  if wp == "wp1":
    x = 0.250
    y = 0.300
    z = 0.350
  if wp == "wp2":
    x = 0.150
    y = 0.200
    z = 0.400
  if wp == "wp3":
    x = 0.400
    y = 0.200
    z = 0.150
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.w = 1.0
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z

  move_group.set_pose_target(pose_goal)

  ## Now, we call the planner to compute the plan and execute it.
  plan = move_group.go(wait=True)
  # Calling `stop()` ensures that there is no residual movement
  move_group.stop()
  # It is always good to clear your targets after planning with poses.
  # Note: there is no equivalent function for clear_joint_value_targets()
  move_group.clear_pose_targets()
  current_pose = move_group.get_current_pose().pose
  return all_close(pose_goal, current_pose, 0.01)

def callback(msg):
  fb = rosplan_dispatch_msgs.msg.ActionFeedback()
  fb.action_id = msg.action_id
  fb.status = "action enabled"
  action_feedback_pub.publish(fb)

  if msg.name == "goto_waypoint":
    for p in msg.parameters:
      if p.key == "to":
        go_to_pose_goal(p.value)
        fb = rosplan_dispatch_msgs.msg.ActionFeedback()
        fb.action_id = msg.action_id
        fb.status = "action achieved"
        action_feedback_pub.publish(fb)


rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", rosplan_dispatch_msgs.msg.ActionDispatch, callback)
rospy.spin()