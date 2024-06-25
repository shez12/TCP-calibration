import rospy,sys
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python")
robot = moveit_commander.RobotCommander()

group_names = robot.get_group_names()
arm = moveit_commander.MoveGroupCommander("manipulator")
states = arm.get_current_joint_values()
goal = states
goal[0]+= 0.2
arm.set_joint_value_target(goal)
arm.go()
arm.stop()