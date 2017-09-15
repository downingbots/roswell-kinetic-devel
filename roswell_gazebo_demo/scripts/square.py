#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# We always import roslib, and load the manifest to handle dependencies
# import roslib; roslib.load_manifest('mini_max_tutorials')

# recall: robots generally take base movement commands on a topic 
#  called "cmd_vel" using a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist

class square:
    """ This example is in the form of a class. """

    def __init__(self):
        """ This is the constructor of our class. """
        # register this function to be called on shutdown
        rospy.on_shutdown(self.cleanup)

        # publish to cmd_vel
        # self.pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher('/base_controller/command', Twist, queue_size=30)
        # give our node/publisher a bit of time to connect
        rospy.loginfo("square: sleep(1)...")
        rospy.sleep(1)

        # use a rate to make sure the bot keeps moving
        r = rospy.Rate(5.0)
        # create a Twist message, fill it in to drive forward a bit
        rospy.loginfo("square: drive fwd1...")
        twist = Twist()
        twist.linear.x = 0.15 
        for i in range(3 * 5):         # 10*5hz = 2sec
          self.pub.publish(twist)
          r.sleep()

        # go forever!
        while not rospy.is_shutdown():
        # for s in range(2):
          for s in range(4):         # 10*5hz = 2sec
            # create a Twist message, fill it in to drive forward
            rospy.loginfo("square: drive fwd...")
            twist = Twist()
            twist.linear.x = 0.0
            self.pub.publish(twist)
            r.sleep()
            twist.linear.x = 0.15
            for i in range(5 * 5):         # 10*5hz = 2sec
                self.pub.publish(twist)
                r.sleep()
            # for i in range(8 * 5):         # 10*5hz = 2sec
            #     r.sleep()
            # create a twist message, fill it in to turn
            rospy.loginfo("square: twist...")
            twist = Twist()
            twist.angular.z = 1.57/16    # 45 deg/s * 2sec = 90 degrees
            for i in range(16 * 5):         # 10*5hz = 2sec
                self.pub.publish(twist)
                r.sleep()
            twist = Twist()
            # rospy.loginfo("stop via direcion change...")
            # twist.linear.x = -0.00001
            # twist.angular.z = 0
            # self.pub.publish(twist)
            # r.sleep()
            # twist.linear.x = 0.00001
            # self.pub.publish(twist)
            # r.sleep()
            # twist.linear.x = 0
            # twist.angular.z = 0
            # rospy.loginfo("stop...")
            # for i in range(16 * 5):         # 10*5hz = 2sec
                # self.pub.publish(twist)
                # r.sleep()

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub.publish(twist)

# if __name__=="__main__":
    # rospy.init_node('square')
    # square()

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("OK for move_base...")

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        rospy.loginfo("send goal for move_base: x %f, y %f, theta %f" %(x, y, theta))
        self.client.wait_for_result()
        rospy.loginfo("result for move_base...")

# Send a trajectory to controller
class TorsoClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso_controller")
        self.client.wait_for_server()
        rospy.loginfo("Waiting complete for torso_controller")

    def move_to(self, position, duration=15.0):
        # trajectory = JointTrajectory()
        # trajectory.joint_names = ["arm_lift_joint",]
        # trajectory.points.append(JointTrajectoryPoint())
        # trajectory.points[0].positions = [position, ]
        # trajectory.points[0].velocities = [0.0,]
        # trajectory.points[0].accelerations = [0.0,]
        # trajectory.points[0].time_from_start = rospy.Duration(duration)
        # follow_goal = FollowJointTrajectoryGoal()
        # follow_goal.trajectory = trajectory
        torso_goal = FollowJointTrajectoryGoal()
        torso_point = JointTrajectoryPoint()
        torso_point.positions = [position,]
        torso_point.time_from_start = rospy.Duration(duration)
        torso_goal.trajectory.points.append(torso_point)
        torso_goal.trajectory.header.stamp = rospy.Time.now()
        torso_goal.trajectory.joint_names = ["arm_lift_joint",]

        self.client.send_goal(torso_goal)
        rospy.loginfo("moveto goal")
        self.client.wait_for_result(rospy.Duration(duration))
        rospy.loginfo("moveto done")

# Point the head using controller
class PointHeadClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        rospy.loginfo("Waiting for head_controller complete...")
        self.pan_tilt_client = actionlib.SimpleActionClient(
                                     "head_controller/follow_joint_trajectory",
                                     FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for pan-tilt head_controller...")
        self.pan_tilt_client.wait_for_server()
        rospy.loginfo("Waiting for pan-tilt head_controller complete...")


    def look_at(self, x, y, z, frame, duration=5.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        rospy.loginfo("look_at goal")
        self.client.wait_for_result(rospy.Duration(5.0))
        rospy.loginfo("look_at done")
   
    def pan_tilt(self, pan, tilt):
        duration = 5
        trajectory = JointTrajectory()
        trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = [pan, tilt]
        trajectory.points[0].velocities = [0.0, 0.0]
        trajectory.points[0].accelerations = [0.0, 0.0]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        self.pan_tilt_client.send_goal(follow_goal)
        rospy.loginfo("pan_tilt goal")
        self.pan_tilt_client.wait_for_result(rospy.Duration(5.0))
        rospy.loginfo("pan_tilt done")

# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server(rospy.Duration(5.0))
        rospy.loginfo("Waiting for %s complete..." % find_topic)

    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        rospy.loginfo("Waiting for sync")
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         wait = False)
        rospy.loginfo("number of objects found: %d", idx)

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         wait = False)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspableCube(self):
        graspable = None
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1:
                rospy.loginfo("no object.")
                continue
            # check size
	    rospy.loginfo("object dimension: %f", obj.object.primitives[0].dimensions[0])
            if obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07:
                continue
            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5:
	        rospy.loginfo("object z dimension: %f", obj.object.primitive_poses[0].position.z)
                continue
            return obj.object, obj.grasps
        # nothing detected
	rospy.loginfo("nothing detected")
        return None, None

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 90 degrees in yaw direction
        l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 1.57)
        places.append(copy.deepcopy(l))
        l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 1.57)
        places.append(copy.deepcopy(l))
        l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 1.57)
        places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_upperarm_roll_joint",
                  "arm_elbow_flex_joint", "arm_wrist_flex_joint", "arm_wrist_roll_joint"]
        # pose = [1.32, 1.40, -0.2, 1.72, 1.66, 0.0]
        pose = [0.00, 0.00, 0.00, 0.00, 0.00, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    torso_action = TorsoClient()
    head_action = PointHeadClient()
    grasping_client = GraspingClient()

    # rospy.loginfo("pan_tilt 0 0")
    # head_action.pan_tilt(0.0, 0.0)
    # move_base.goto(0.0, 0.0, 0.0)
    # move_base.goto(0, 0, 0)
    # move_base.goto(1, 0,  0)
    # rospy.loginfo("square: sleep(3)...")
    # rospy.sleep(3)
    # move_base.goto(.5, .5, 0)
    # rospy.loginfo("square: sleep(3)...")
    # rospy.sleep(3)
    # move_base.goto(2, 2, 0)
    # rospy.loginfo("square: sleep(3)...")
    # rospy.sleep(3)
    # move_base.goto(2.250, 3.118, 0.0)
    # rospy.loginfo("square: sleep(3)...")
    # rospy.sleep(3)
    # move_base.goto(2.750, 3.118, 0.0)
    # rospy.loginfo("square: sleep(3)...")
    # rospy.sleep(3)
    # move_base.goto(-1, 1, 1.57)
    rospy.loginfo("making a square.")
    square_move = square()
    # move_base.goto(-3.53, 3.75, 1.57)
    # move_base.goto(2.250, 2.618, 0.0)
    # Raise the torso using just a controller
##    while True:
#    rospy.loginfo("Raising torso 0...")
    #torso_action.move_to(0)
    #rospy.loginfo("Lowering torso -.4 ...")
    #torso_action.move_to(-0.4)
    #rospy.loginfo("Lowering torso .4 ...")
    #torso_action.move_to(0.4)
    #rospy.loginfo("raising torso 0...")
    ## grasping_client.tuck()
    #torso_action.move_to(0)


    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
    # rospy.loginfo("Moving to table...")
    # move_base.goto(2.250, 2.618, 0.0)
    # move_base.goto(2.750, 3.118, 0.0)
    # move_base.goto(2.250, 3.118, 0.0)
    # move_base.goto(2.80, 3.118, 0.0)
    # move_base.goto(2.80, 3.118, 0.0)
    # move_base.goto(2.7, 3.118, 0.0)

    # Raise the torso using just a controller
    rospy.loginfo("Raising torso...")
    # torso_action.move_to([0.4, ])
    # torso_action.move_to([-0.1, ])
    torso_action.move_to(-0.1)

    # Point the head at the cube we want to pick
    head_action.look_at(3.7, 3.18, 0.0, "map")
    # <pose>3.7 2.85 0.83 0 0 0</pose>
    rospy.loginfo("look_at 2.75 1.57")
    head_action.look_at(3.75, 3.18, 0.0, "map")
    # rospy.loginfo("pan_tilt 0 1.57")
    # head_action.pan_tilt(0.0, 1.57)


    # Get block to pick
    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        grasping_client.updateScene()
        cube, grasps = grasping_client.getGraspableCube()
        if cube == None:
            rospy.logwarn("Perception failed.")
            continue

        # Pick the block
        if grasping_client.pick(cube, grasps):
            break
        rospy.logwarn("Grasping failed.")

    # Tuck the arm
    grasping_client.tuck()

    # Lower torso
    rospy.loginfo("Lowering torso...")
    # torso_action.move_to([0.0, ])
    torso_action.move_to([-0.591, ])

    # Move to second table
    rospy.loginfo("Moving to second table...")
    move_base.goto(-3.53, 3.75, 1.57)
    move_base.goto(-3.53, 4.15, 1.57)

    # Raise the torso using just a controller
    rospy.loginfo("Raising torso...")
    # torso_action.move_to([0.4, ])
    torso_action.move_to([-0.1, ])

    # rospy.loginfo("head 1.57 0 -1.57 ")
    # head_action.look_at(1.57, 0.0, 0.0, "map")
    # head_action.look_at(1.57, 0.0, -1.57, "map")

    # Place the block
    while not rospy.is_shutdown():
        rospy.loginfo("Placing object...")
        pose = PoseStamped()
        pose.pose = cube.primitive_poses[0]
        pose.pose.position.z += 0.05
        pose.header.frame_id = cube.header.frame_id
        if grasping_client.place(cube, pose):
            break
        rospy.logwarn("Placing failed.")

    # Tuck the arm, lower the torso
    grasping_client.tuck()
    # torso_action.move_to([0.0, ])
    torso_action.move_to([-0.591, ])
