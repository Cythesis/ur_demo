#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from math import radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from ar_track_alvar_msgs.msg import AlvarMarkers
import time 
arFound = 0

def all_close(goal, actual, tolerance):
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

class MoveGroupPythonInterfaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()
    r = rospy.Rate(10)
    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self,msg):
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    if msg == 0:
		joint_goal[0] = 0
		joint_goal[1] = radians(-90)
		joint_goal[2] = 0
    elif msg == 1:
		joint_goal[0] = radians(49.48)
		joint_goal[1] = radians(-98.66)
		joint_goal[2] = radians(132.56)
    elif msg == 2:
    	joint_goal[0] = radians(-45.77)
    	joint_goal[1] = radians(-70.59)
    	joint_goal[2] = radians(95.41)
    elif msg == 3:
    	joint_goal[0] = 0
    	joint_goal[1] = radians(-90)
    	joint_goal[2] = 0

    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  # def go_to_pose_goal(self, msg):
  #   move_group = self.move_group
  #   pose_goal = self.move_group.get_current_pose().pose
  #   # pose_goal = geometry_msgs.msg.Pose()
  #   # pose_goal.orientation.w = 1.0
  #   # pose_goal.position.x = msg.markers[0].pose.pose.position.x
  #   # pose_goal.position.y = msg.markers[0].pose.pose.position.y
  #   try:
  #       pose_goal.position.z = msg.markers[0].pose.pose.position.z
  #   except:
  #       print("No markers found!")
  #   move_group.set_pose_target(pose_goal)

  #   plan = move_group.go(wait=True)
  #   move_group.stop()
  #   move_group.clear_pose_targets()

  #   current_pose = self.move_group.get_current_pose().pose
  #   print("Current z pose is: " + str(current_pose.position.z))
  #   try:
  #       print("Target z pose is: " + str(msg.markers[0].pose.pose.position.z))
  #   except:
  #       print("")
  #   print("")
  #   return all_close(pose_goal, current_pose, 0.01)



def warningMessage():
	print("-------------")
	print("Checking in 5")
	print("-------------")
	time.sleep(1)
	print("-------------")
	print("Checking in 4")
	print("-------------")
	time.sleep(1)
	print("-------------")
	print("Checking in 3")
	print("-------------")
	time.sleep(1)
	print("-------------")
	print("Checking in 2")
	print("-------------")
	time.sleep(1)
	print("-------------")
	print("Checking in 1")
	print("-------------")
	time.sleep(1)

def callback(data):
	global arFound
	try:
    		t = data.markers[0].pose.pose.position.x
    		arFound = 1
    		print("AR Detected")
    	except:
    		arFound = 0
    		print("No AR Detected")

class homePos(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arFound','arNotFound'])
		
	def execute(self, userdata):
		global tutorial
		MoveGroupPythonInterfaceTutorial.go_to_joint_state(tutorial, 0);
		warningMessage()
		if arFound == 1:
			return 'arFound'
		else:
			return 'arNotFound'
		

class pos1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arFound','arNotFound'])

	def execute(self, userdata):
		global tutorial
		MoveGroupPythonInterfaceTutorial.go_to_joint_state(tutorial, 1);
		warningMessage()
		if arFound == 1:
			return 'arFound'
		else:
			return 'arNotFound'


class pos2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arFound','arNotFound'])

	def execute(self, userdata):
		global tutorial
		MoveGroupPythonInterfaceTutorial.go_to_joint_state(tutorial, 2);
		warningMessage()
		if arFound == 1:
			return 'arFound'
		else:
			return 'arNotFound'

class pos3(smach.State):
	global tutorial
	def __init__(self):
		smach.State.__init__(self, outcomes=['arFound','arNotFound'])

	def execute(self, userdata):
		MoveGroupPythonInterfaceTutorial.go_to_joint_state(tutorial, 3);
		warningMessage()
		if arFound == 1:
			return 'arFound'
		else:
			return 'arNotFound'

def main():
	global tutorial
	rospy.init_node('ur_ar_smach_machine')
	rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback, queue_size=1)
	tutorial = MoveGroupPythonInterfaceTutorial()
	sm = smach.StateMachine(outcomes=['Success','Failed'])

	with sm:
		smach.StateMachine.add('homePos', homePos(), transitions={'arFound':'pos1','arNotFound':'Failed'})
		smach.StateMachine.add('pos1', pos1(), transitions={'arFound':'pos2','arNotFound':'homePos'})
		smach.StateMachine.add('pos2', pos2(), transitions={'arFound':'pos3','arNotFound':'pos1'})
		smach.StateMachine.add('pos3', pos3(), transitions={'arFound':'Success','arNotFound':'pos2'})
	
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	outcome = sm.execute()

if __name__ == '__main__':
	main()