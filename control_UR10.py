import rospy
from pynput import keyboard
from moveit_commander import RobotCommander, MoveGroupCommander
import UR10_toolbox as ik  # Assuming this is where your IK function is defined
import os

# Global variables to store the latest joint state and positions
TCP = [0.03896421, -0.22570285, 0.37671283]#roughly the tool center point
movement_step_size = 0.0005
home_position = []
fixed_joint_positions = []

def move_to_specific_point(joint_positions):
    move_group.set_joint_value_target(joint_positions)
    print("moving to joint point", joint_positions, "...")
    move_group.go()
    move_group.stop()

def move_robot(direction):
    joint_positions = move_group.get_current_joint_values()
    if joint_positions is None:
        rospy.logwarn("Joint positions are not available yet.")
        return
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

    q = ik.UR10_IK(joint_positions, T, TCP)
    move_group.set_joint_value_target(q)
    move_group.go()
    print("moving...")
    move_group.stop()


def on_press(key):
    global movement_step_size
    key_mapping = {
        'q': "x+", 'w': "x-", 'a': "y+", 's': "y-", 'z': "z+", 'x': "z-",
        'h': home_position, 'f': fixed_joint_positions
    }
    try:
        if key == keyboard.Key.esc:
            print("esc is pressed, exit...")
            rospy.signal_shutdown("closed...")
            os._exit(0)
        elif hasattr(key, 'char'):
            if key.char in key_mapping:
                if isinstance(key_mapping[key.char], str):
                    move_robot(key_mapping[key.char])
                else:
                    move_to_specific_point(key_mapping[key.char])
            elif key.char == 'o':
                movement_step_size = max(0.0001, movement_step_size - 0.0001)
                print("current movement step size: ", movement_step_size)
            elif key.char == 'p':
                movement_step_size = min(0.001, movement_step_size + 0.0001)
                print("current movement step size: ", movement_step_size)
            elif key.char == 'm':
                with open('joint_states.txt', 'a') as file:
                    file.write("Joint Positions: {}\n\n".format(move_group.get_current_joint_values()))
                print("save point...")
    except AttributeError:
        print("Invalid key pressed.")
        raise

def main():
    global move_group, robot
    rospy.init_node('TCP_calibration')
    robot = RobotCommander()
    move_group = MoveGroupCommander("manipulator")
    rospy.loginfo("Use Q/W/A/S/Z/X to move, H to home, F for fixed positions, O/P to adjust step size, M to save positions, ESC to exit.")
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    rospy.spin()
    listener.stop()

if __name__ == '__main__':
    main()
