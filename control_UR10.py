
import rospy
from pynput import keyboard
from moveit_commander import RobotCommander, MoveGroupCommander
import UR10IK as ik  # Assuming this is where your IK function is defined

# Global variables to store the latest joint state and positions
joint_positions = None
TCP = [0, 0, 0]
# Set the movement step size (can be adjusted to control speed)
global movement_step_size 
movement_step_size= 0.0005


def move_robot(direction):

    joint_positions = move_group.get_current_joint_values()

    if joint_positions is None:
        rospy.logwarn("Joint positions are not available yet.")
        return

    # Calculate the target pose based on current joint positions and direction
    T = ik.UR10_FK(joint_positions, TCP)
    if direction == "x+":
        T[0, 3] += movement_step_size
    elif direction == "x-":
        T[0, 3] -= movement_step_size
    elif direction == "y+":
        T[1, 3] += movement_step_size
    elif direction == "y-":
        T[1, 3] -= movement_step_size
    elif direction == "z+":
        T[2, 3] += movement_step_size
    elif direction == "z-":
        T[2, 3] -= movement_step_size

    # Perform inverse kinematics to get joint positions for the target pose
    q = ik.UR10_IK(joint_positions, T, TCP)
    print(joint_positions)
    print(q)

    move_group.set_joint_value_target(q)
    move_group.go()
    move_group.stop()

    print("moving...")

def on_press(key):
    try:
        if key.char.lower() == "q":
            move_robot("x+")
        elif key.char.lower() == "w":
            move_robot("x-")
        elif key.char.lower() == "a":
            move_robot("y+")
        elif key.char.lower() == "s":
            move_robot("y-")
        elif key.char.lower() == "z":
            move_robot("z+")
        elif key.char.lower() == "x":
            move_robot("z-")
        elif key == keyboard.Key.esc:
            rospy.signal_shutdown("closed...")
        elif key.char.lower() == "m":
            with open('joint_states.txt', 'a') as file:
                file.write("Joint Positions: {}\n\n".format(move_group.get_current_joint_values()))
            print("save point...")

    except AttributeError:
        pass

def main():
    global move_group
    global robot


    rospy.init_node('TCP_calibration')

    # Initialize MoveIt commander objects
    robot = RobotCommander()
    move_group = MoveGroupCommander("manipulator")
    



    rospy.loginfo("Press Q to move forward in x-axis; Press W to move backward in x-axis; A to move forward in y-axis; S to move backward in y-axis; Z to move forward in z-axis; X to move backward in z-axis.")

    # Create a keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    rospy.spin()

    # Ensure the listener is properly stopped
    listener.stop()

if __name__ == '__main__':
    main()


