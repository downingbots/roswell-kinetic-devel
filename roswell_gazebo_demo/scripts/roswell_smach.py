#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

state
  table_list
    add_table
    remove_table
       adds pointer to cube/boxes
  cube_list
    add_cube
       checks if already there
    remove_cube
  box_list
    add_box(location, state - on_table, on_ground, in_box?, in_box ptr)
    remove_box

  state_list
    add_state
    prev_state

  cube_in_grasp (true/false, cube_ptr)
   

nav_utiltities
  face_table()
  goto_table_edge
  backoff_table_edge
  SLAM_with_object_detection_and_tracking


vision_
  scan_room
  scan_for obstacles
  scan_for obstacle and objects
     Look for objects
     when looking in right direction (near table centroid when at distance)
       Look for table
       if table found, 
         look for cube on table
         Look for box on table
  
  

object_utilities
  look_for_object_in_grasp
       returns true false
  look_for_object_on_close_table
       returns box cube other none
  look_for_object_on_far_table
       returns box cube other none
       returns list & the locations
  look_for_object_on_floor
       returns box cube other none
  look_for_possible_table
       returns box cube other none
  


# define state Foo
class find_table(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_table', ' possible_table', 'no_table'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

# define state Bar
class navigate_to_table(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        face_table()
        rospy.loginfo('Executing state BAR')
        return 'outcome2'
        
class explore_map(smach.State):

class navigate_to_object(smach.State):

class pick_up_object(smach.State):

# main
def main():
    rospy.init_node('roswell_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outCome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FIND TABLE', find_table():
                               transitions={'found_table':'navigate_to_table', 
                                            'possible_table':'navigate_to_table'})
                                            'no_table':'explore'})
        smach.StateMachine.add('NAVIGATE TO TABLE', navigate_to_table(), 
                               transitions={'outcome2':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()

