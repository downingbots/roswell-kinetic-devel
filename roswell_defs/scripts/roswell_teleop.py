#!/usr/bin/python
#
#  send tele-operation commands to pilot from an HKS racing controller
#
#   Copyright (C) 2011 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'maxwell_defs'

# standard Python packages
import sys
import math
import trajectory_msgs.msg

import roslib 
roslib.load_manifest(PKG_NAME)
from std_msgs.msg import Float64
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetSpeed, SetTorqueLimit
import actionlib
from actionlib_msgs.msg import *
from control_msgs.msg import *
#from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, conversions
#from moveit_msgs.msg import RobotTrajectory, Grasp
from moveit_msgs.msg import *
from maxwell_defs.srv import ReturnJointStates
from sound_utilities import SpeechEngine
#from parallel_gripper_controller import ParallelGripperController


# In ROS Electric, the Joy message gained a header and moved to
# sensor_msgs.  Use that if it's available.
try:
    from sensor_msgs.msg import Joy
except ImportError:
    from joy.msg import Joy

def clamp(minimum, value, maximum):
    "constrain value to the range [minimum .. maximum]"
    return max(minimum, min(value, maximum))

class JoyNode():
    "Pr2Lite joystick tele-operation node."

    def __init__(self):
        "JoyNode constructor"
        print "joystick constructor"

        # PRLite Joystick controls

#       # Modes -> Front buttons
        self.arm_mode = 5         # front bottom right
        self.servo_arm_mode = 4   # front bottom left
        self.head_mode = 7        # front top left
        # self.body_mode = 6        # front top right
        self.nav_mode = 6         # front top left
        self.prev_mode = 0        # no mode set
#
#       # BUTTON mapping
        self.cur_pos = 0        
        self.pose_incr = 9        # start
        self.pose_decr = 8        # select
#   
#       # digital Joystick Axes
        self.torso_up_down = 5    # cross: 1 = up, -1 = down
        self.shoulder_tilt = 5    # cross: 1 = up, -1 = down
        self.shoulder_pan = 4     # cross: 1 = right -1 = left 

#
#       # analog joysticks Axes; mode dependent
#
        # joystick -head
        # head controller
        self.head_tilt = 1        # left Joy: 1 = up, -1 = down
        self.head_pan = 0         # left Joy: 1 = right -1 = left 
        # head servo
        self.head_pan_servo = 4   # cross (right/left)
        self.head_tilt_servo = 5  # cross (up/down)
	self.arm_lift_up = 3      # square
	self.arm_lift_down = 1    # circle

        # joystick - base
        self.move_fwd_bck = 1     # left Joy: 1 = fwd, -1 = back
        self.rot_left_right = 0   # left Joy: 1 = right, -1 = left
        self.move_left_right = 3  # right Joy: 1 = right -1 = left 
        self.rot_right_left = 2   # right Joy: 1 = fwd, -1 = back

        self.follow_mode = 8      # select

	# servo mode
        self.arm_shoulder_pan =  4               # cross: 1 = right -1 = left
        self.arm_shoulder_tilt = 5               # cross: 1 = up, -1 = down
        self.arm_upperarm_roll = 1               # left joy (left / right) 
        self.elbow_flex = 0                      # left joy (up/down)
        self.wrist_flex = 2                      # right joy (up/down)
	self.arm_lift = 3                        # right joy (left/right)
        # self.wrist_roll

	# arm and servo mode
        self.wrist_roll_clock = 0        # triangle
        self.wrist_roll_counterclock = 2 # X
        self.gripper_close = 3           # square
        self.gripper_open = 1            # circle

	# arm mode
        # cross - gripper orientation
        self.elbow_pan = 4               # cross: 1 = right -1 = left
        self.wrist_flex = 5              # cross: 1 = up, -1 = down
        # joystick - gripper movement
        self.wrist_up_down = 1       # left Joy: 1 = up, -1 = down
        self.wrist_fwd_bck = 2       # right Joy: 1 = up, -1 = down
        self.wrist_right_left = 3    # right Joy: 1 = right -1 = left 

        # Velo Testing
        # buttons
#       self.speed_up = 1         # "0"
#       self.speed_down = 3       # square
#       self.torque_up = 0        # triangle
#       self.torque_down = 2      # X
 
        # Arm Poses
        self.pose_arm_up = 0
        self.pose_arm_down   = 1   
        self.pose_arm_out = 2   

        # Head Mode
        self.head_joy        = 1   
        self.head_look_up    = 2   
        self.head_look_down  = 3   
        self.head_shake_no   = 4   
        self.head_nod_yes    = 5   
        self.head_hand = 7   
        self.head_pos = self.head_joy
        self.phg = PointHeadGoal()

        # analog joysticks axes
        self.position = 2         # right joy (up / down)

        # moveit and higher-level controllers (arm_mode, headmode, and nav mode)
        head_controller = '/head_traj_controller/point_head_action'
        base_controller = '/base_controller/command'
        # arm_controller = '/arm_controller/command'
        gripper_controller = '/gripper_controller/gripper_action'

	
        # joint-level controllers (servo_mode)
        head_pan_controller = '/head_pan_joint/command'
        head_tilt_controller = '/head_tilt_joint/command'
        arm_lift_controller = '/arm_lift_controller/command'
        arm_shoulder_pan_controller = '/arm_shoulder_pan_joint/command'
        arm_shoulder_tilt_controller = '/arm_shoulder_tilt_joint/command'
        arm_upperarm_roll_controller = '/arm_upperarm_roll_joint/command'
        elbow_flex_controller = '/arm_elbow_flex_joint/command'
        wrist_flex_controller = '/arm_wrist_flex_joint/command'
        wrist_roll_controller = '/arm_wrist_roll_joint/command'

        # initialize ROS topics 
        rospy.init_node('roswell_teleop')

        self.torque = 500
        if self.torque > 1023:
           self.torque = 1023
        if self.torque < 0:
           self.torque = 0

	# servo topics
        self.arm_lift_client = rospy.Publisher(arm_lift_controller, Float64, queue_size=10)
	self.arm_shoulder_pan_client = rospy.Publisher(arm_shoulder_pan_controller, Float64, queue_size=10)
	self.arm_shoulder_tilt_client = rospy.Publisher(arm_shoulder_tilt_controller, Float64, queue_size=10)
        self.upperarm_roll_client = rospy.Publisher(arm_upperarm_roll_controller, Float64, queue_size=10)
        self.elbow_flex_client = rospy.Publisher(elbow_flex_controller, Float64, queue_size=10)
        self.wrist_flex_client = rospy.Publisher(wrist_flex_controller, Float64, queue_size=10)
        self.wrist_roll_client = rospy.Publisher(wrist_roll_controller, Float64, queue_size=10)

        print "wait for head controller"
        self.head_client = actionlib.SimpleActionClient(
                  head_controller, PointHeadAction)
        self.head_client.wait_for_server()

	self.head_pan_client = rospy.Publisher(head_pan_controller, Float64, queue_size=10)
	self.head_tilt_client = rospy.Publisher(head_tilt_controller, Float64, queue_size=10)

        self.cmd_vel = rospy.Publisher(base_controller, Twist, queue_size=10)

        #TODO: moveit!
        print "arm and torso moveit commander"
        rospy.loginfo("roswell_teleop: arm moveit commander")
        try:
          self.arm_group = MoveGroupCommander("arm")
        except rospy.ServiceException, e:
          self.arm_group = MoveGroupCommander("arm")

        rospy.loginfo("roswell_teleop: speech engine")
        self.se = SpeechEngine()

        print "subscribe to joystick"
        self.joy = rospy.Subscriber('joy', Joy, self.joyCallback)
        print "joystick ready"
        self.se.say( "joystick ready")

        self.forward = 0
        self.turn = 0
        self.X = 0
        self.Y = 0
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist,  queue_size=10)

    def get_joint_state(self,name):
        joint_names = [name]
        rospy.wait_for_service("return_joint_states")
        try:
          s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
          resp = s(joint_names)
        except rospy.ServiceException, e:
          print "error when calling return_joint_states: %s"%e
          sys.exit(1)
        for (ind, joint_name) in enumerate(joint_names):
          if joint_name == name:
            pos = resp.position[ind]
          elif(not resp.found[ind]):
            print "joint %s not found!"%joint_name
        return pos

    def handle_wrist_and_gripper(self, joy):
        if joy.buttons[self.arm_mode] or joy.buttons[self.servo_mode]:
            wrist_delta = .15
            if joy.buttons[self.wrist_roll_clock]:
              wrist_roll_pos = self.get_joint_state('wrist_roll_joint')
              wrist_roll_pos = wrist_roll_pos - wrist_delta
              self.wrist_roll_client.publish(wrist_roll_pos)
            if joy.buttons[self.wrist_roll_counterclock]:
              wrist_roll_pos = self.get_joint_state('wrist_roll_joint')
              wrist_roll_pos = wrist_roll_pos + wrist_delta
              self.wrist_roll_client.publish(wrist_roll_pos)
            if joy.axes[self.wrist_flex] != 0:
              wrist_flex_pos = self.get_joint_state('wrist_flex_joint')
              wrist_flex_pos = wrist_flex_pos - joy.axes[self.wrist_flex]*wrist_delta
              self.wrist_flex_client.publish(wrist_flex_pos)
            if joy.buttons[self.elbow_pan] != 0:
              elbow_pan_pos = self.get_joint_state('elbow_pan_joint')
              elbow_pan_pos = elbow_pan_pos - joy.axes[self.elbow_pan]*wrist_delta
              self.elbow_pan_client.publish(elbow_pan_pos)
            # TODO: Joystick Arm control
            #
            if joy.buttons[self.gripper_open]:
              gripper_delta = .005
              self.gripper_pos = self.gripper_pos + gripper_delta
              if self.gripper_pos > self.max_gripper_pos:
                self.gripper_pos = self.max_gripper_pos
              print "left gripper " + str(self.gripper_pos)
              goal = Pr2GripperCommandGoal()
              goal.command.position = self.gripper_pos
              goal.command.max_effort = 1000
              self.gripper_client.send_goal(goal)
              self.gripper_client.wait_for_result(rospy.Duration(1.0))
            if joy.buttons[self.gripper_close]:
              gripper_delta = .005
              self.gripper_pos = self.gripper_pos - gripper_delta
              if self.gripper_pos < 0:
                self.gripper_pos = 0
              print "left gripper " + str(self.gripper_pos)
              goal = Pr2GripperCommandGoal()
              goal.command.position = self.gripper_pos
              goal.command.max_effort = 1000
              self.gripper_client.send_goal(goal)
              self.gripper_client.wait_for_result(rospy.Duration(1.0))
    
    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        #rospy.logdebug('joystick input:\n' + str(joy))
        rospy.loginfo('joystick input:\n' + str(joy))
        print "joystick input:" 
        print joy

	vel = Twist()
	vel.angular.z = 0
	vel.linear.x = 0
	vel.linear.y = 0
        if joy.buttons[self.arm_mode]:
            if self.prev_mode != self.arm_mode:
              self.prev_mode = self.arm_mode
              self.se.say( "arm mode")
            new_pos = False
            if joy.buttons[self.pose_incr]:
              self.cur_pos = self.cur_pos + 1
              if self.cur_pos > self.pose_stretch:
                self.cur_pos = self.pose_move_aside
              new_pos = True
            if joy.buttons[self.pose_decr]:
              self.cur_pos = self.cur_pos - 1
              if self.cur_pos < self.pose_move_aside:
                self.cur_pos = self.pose_stretch
              new_pos = True
            print "new pos " + str(new_pos)
            # TODO: have head follow arm?
            if new_pos and self.cur_pos == self.pose_arm_up:
              print "arm_up"
              self.se.say( "arm_up")
              self.arm_group.allow_replanning(True)
              self.arm_group.set_named_target("arm_up")
              # planned = self.arm_group.plan()
              if self.arm_group.go() == False:
                self.se.say( "Error")
            elif new_pos and self.cur_pos == self.pose_arm_down:
              print "arm_down"
              self.se.say( "arm down")
              self.arm_group.allow_replanning(True)
              self.arm_group.set_named_target("arm_down")
              # planned = self.arm_group.plan()
              if self.arm_group.go() == False:
                self.se.say( "Error")
            elif new_pos and self.cur_pos == self.pose_arm_out:
              print "arm_out"
              self.se.say( "arm out")
              self.arm_group.allow_replanning(True)
              self.arm_group.set_named_target("arm_out")
              # planned = self.arm_group.plan()
              if self.arm_group.go() == False:
                self.se.say( "Error")
              # planned = self.arm_group.plan()
              if self.arm_group.go() == False:
                self.se.say( "Error")
            self.handle_wrist_and_gripper(joy)
            threshold = 0.1
	    if math.fabs(joy.axes[self.wrist_left]) > threshold or math.fabs(joy.axes[self.wrist_fwd_bck]) > threshold or math.fabs(joy.axes[self.wrist_up_down]) > threshold:
              return
              self.se.say( "joystick control of arm")
              print "joystick arm control"
              currentrf = self.arm_group.get_pose_reference_frame()
              self.arm_group.set_pose_reference_frame(currentrf)
              self.arm_group.allow_replanning(False)
              # Comment out replanning:
              # self.arm_group.allow_replanning(True)
              # With replanning, the joystick arm control is frequently called
              # with a new status.   The planning never executes anything 
              # because it is always replanning and never executing.  
              # For now, no replannng and use blocking callbacks.
#following lines core dump
#              ee_link = self.arm_group.get_end_effector_link()
#              newpose = self.arm_group.get_current_pose(ee_link).pose
#              if self.arm_group.has_end_effector_link():
#                print "eef: " +  self.arm_group.get_end_effector_link()
#              # newposerpy = self.arm_group.get_current_rpy()
#              arm_delta = 0.45
#              newpose.position.x = newpose.position.x-arm_delta * joy.axes[self.wrist_left]
#              newpose.position.y = newpose.position.y-arm_delta * joy.axes[self.wrist_fwd_bck]
#              newpose.position.z = newpose.position.z-arm_delta * joy.axes[self.wrist_up_down]
#              # print newpose   
#              print('Setting a new target pose..')
#              self.arm_group.set_pose_target(newpose, ee_link)
              self.arm_group.set_start_state_to_current_state()
              planned = self.arm_group.plan()
              print ('Joint interpolated plan:')
              print planned
              self.arm_group.go()
              # self.arm_group.execute(planned)
              # scene = PlanningSceneInterface()
        elif joy.buttons[self.servo_mode]:
            if self.prev_mode != self.servo_mode:
              self.prev_mode = self.servo_mode
              self.se.say( "servo mode")
            print "new pos " + str(new_pos)
            self.handle_wrist_and_gripper(joy)
            servo_delta = .15
            if joy.buttons[self.arm_shoulder_pan]:
              arm_shoulder_pan_pos = self.get_joint_state('arm_shoulder_pan_joint')
              arm_shoulder_pan_pos = arm_shoulder_pan_pos - servo_delta
              self.arm_shoulder_pan_client.publish(arm_shoulder_pan_pos)
            if joy.buttons[self.arm_shoulder_tilt]:
              arm_shoulder_tilt_pos = self.get_joint_state('arm_shoulder_tilt_joint')
              arm_shoulder_tilt_pos = arm_shoulder_tilt_pos - servo_delta
              self.arm_shoulder_tilt_client.publish(arm_shoulder_tilt_pos)
            if joy.buttons[self.arm_upperarm_roll]:
              arm_upperarm_roll_pos = self.get_joint_state('arm_upperarm_roll_joint')
              arm_upperarm_roll_pos = arm_upperarm_roll_pos - servo_delta
              self.arm_upperarm_roll_client.publish(arm_upperarm_roll_pos)
            if joy.buttons[self.elbow_flex]:
              elbow_flex_pos = self.get_joint_state('elbow_flex_joint')
              elbow_flex_pos = elbow_flex_pos - servo_delta
              self.elbow_flex_client.publish(elbow_flex_pos)
            if joy.buttons[self.wrist_flex]:
              wrist_flex_pos = self.get_joint_state('wrist_flex_joint')
              wrist_flex_pos = wrist_flex_pos - servo_delta
              self.wrist_flex_client.publish(wrist_flex_pos)
            if joy.buttons[self.arm_lift]:
              arm_lift_pos = self.get_joint_state('arm_lift_joint')
              arm_lift_pos = arm_lift_pos - servo_delta
              self.arm_lift_client.publish(arm_lift_pos)
        elif joy.buttons[self.head_mode]:
            if self.prev_mode != self.head_mode:
              self.prev_mode = self.head_mode
              self.se.say( "head mode")

            level_head = 1
            head_mv_tm = .75
            new_pos = False
            if joy.buttons[self.pose_incr]:
              self.head_pos = self.head_pos + 1
              if self.head_pos > self.head_hand:
                self.head_pos = self.head_joy
              new_pos = True
            if joy.buttons[self.pose_decr]:
              self.head_pos = self.head_pos - 1
              if self.head_pos < self.head_joy:
                self.head_pos = self.head_hand
              new_pos = True
            if self.head_pos == self.head_joy:
              # do every loop, not just with new pos
              print "head_joy"
              head_delta = .15
              cur_head_pan = self.get_joint_state('head_pan_joint')
              cur_head_tilt = self.get_joint_state('head_tilt_joint')
              print "cur head tilt %f !"%cur_head_tilt
              print "cur head pan %f !"%cur_head_pan
              threshold = 0.1
              # if math.fabs(joy.axes[self.head_pan]) > threshold or math.fabs(joy.axes[self.head_tilt]) > threshold:
              if True:
                # 3.14 / 2 = 1.57
                new_head_pan = joy.axes[self.head_pan] - 3.14
                new_head_tilt = joy.axes[self.head_tilt] - 1.57
                self.phg.target.header.frame_id = 'head_pan_link'
                self.phg.pointing_frame = "kinect_depth_optical_frame"
                self.phg.target.point.x = 5* math.sin(new_head_tilt) * math.cos(new_head_pan)
                self.phg.target.point.y = 5* math.sin(new_head_tilt) * math.sin(new_head_pan)
                self.phg.target.point.z = 5* math.cos(new_head_tilt) 
                print "head x %f !"%self.phg.target.point.x
                print "head y %f !"%self.phg.target.point.y
                print "head z %f !"%self.phg.target.point.z
                self.phg.min_duration = rospy.Duration(1.0)
                self.head_client.send_goal(self.phg)
                self.head_client.wait_for_result()
                # rospy.sleep(head_mv_tm)
            elif new_pos and self.head_pos == self.head_hand:
              print "head_hand"
              self.phg.target.header.frame_id = 'wrist_roll_link'
              self.phg.target.point.x = 0
              self.phg.target.point.y = 0
              self.phg.target.point.z = 0
              self.phg.pointing_frame = "kinect_depth_optical_frame"
            elif new_pos and self.head_pos == self.head_look_up:
              print "head_look_up"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame"
              self.phg.target.point.x = 5
              self.phg.target.point.y = 0
              self.phg.target.point.z = level_head
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
            elif new_pos and self.head_pos == self.head_look_down:
              print "head_look_down"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame"
              self.phg.target.point.x = 2
              self.phg.target.point.y = 0
              self.phg.target.point.z = 0
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
            elif new_pos and self.head_pos == self.head_shake_no:
              print "head_shake_no"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame"
              self.phg.target.point.x = 5
              self.phg.target.point.y = 0
              self.phg.target.point.z = level_head
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = 1
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = -1
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = 1
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = -1
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = 0
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
            elif new_pos and self.head_pos == self.head_nod_yes:
              print "nod_yes"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame"
              self.phg.target.point.x = 5
              self.phg.target.point.y = 0
              self.phg.target.point.z = level_head
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.x = 2
              self.phg.target.point.z = 0
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.x = 5
              self.phg.target.point.z = level_head
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.x = 2
              self.phg.target.point.z = 0
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.x = 5
              self.phg.target.point.z = level_head
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()

        elif joy.buttons[self.head_mode]:
            # TODO: move to tuck mode?
            # TODO: have head look down/forward?
            # TODO: follow_mode
            if self.prev_mode != self.head_mode:
              self.prev_mode = self.head_mode
            self.phg.target.point.x = joy.axes[self.rot_left_right]
            self.phg.target.point.z = level_head
            self.head_client.send_goal(self.phg)
            self.head_client.wait_for_result()
            self.se.say( "head mode")
        elif joy.buttons[self.nav_mode]:
            # a_scale = 0.9
            a_scale = 0.5
            l_scale = 0.3
            threshold = 0.1
	    if math.fabs(joy.axes[self.move_fwd_bck]) > threshold or math.fabs(joy.axes[self.rot_left_right]) > threshold:
	      vel.angular.z = -1.0*a_scale*joy.axes[self.rot_left_right]
	      vel.linear.x = l_scale*joy.axes[self.move_fwd_bck]
	      vel.linear.y = 0
	    elif math.fabs(joy.axes[self.move_left_right]) > threshold or math.fabs(joy.axes[self.rot_right_left]) > threshold:
	      # for horizontal, roles of linear and angular are switched
	      vel.angular.z = -1.0*a_scale*joy.axes[self.rot_right_left]
	      vel.linear.x = 0
	      vel.linear.y = l_scale*joy.axes[self.move_left_right]
	    else:
	      vel.linear.x = 0
	      vel.linear.y = 0
	      vel.linear.z = 0


		# self.deadman_pressed = msg.buttons[self.axis_deadman] > 0
		# self.twist.linear.x = self.scale_linear * msg.axes[self.axis_linear]
		# self.twist.angular.z = self.scale_angular * msg.axes[self.axis_angular]
		# self.last_message = rospy.Time.now()
	      
        self.cmd_vel.publish(vel)

def main():
    joynode = JoyNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
    rospy.loginfo('joystick vehicle controller finished')

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())

