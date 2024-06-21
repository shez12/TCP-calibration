import rospy
from sensor_msgs.msg import JointState
from pynput import keyboard
import UR10IK as ik

# Global variables to store the latest joint state and positions
current_joint_state = None
joint_positions = None
TCP = [0, 0, 0]

def joint_state_callback(msg):
    global current_joint_state
    global joint_positions
    current_joint_state = msg
    joint_positions = list(current_joint_state.position)
    joint_positions[0], joint_positions[2] = joint_positions[2], joint_positions[0]

def move_robot(direction):
    if joint_positions is None:
        rospy.logwarn("Joint positions are not available yet.")
        return

    T = ik.UR10_FK(joint_positions, TCP)
    if direction == "x+":
        T[0, 3] += 0.001
    elif direction == "x-":
        T[0, 3] -= 0.001
    elif direction == "y+":
        T[1, 3] += 0.001
    elif direction == "y-":
        T[1, 3] -= 0.001
    elif direction == "z+":
        T[2, 3] += 0.001
    elif direction == "z-":
        T[2, 3] -= 0.001

    q = ik.UR10_IK(joint_positions, T, TCP)

    # Send joint angles
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joint_state_msg.position = q

    # Publish the joint state message
    pub.publish(joint_state_msg)

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
    except AttributeError:
        pass

def main():
    global pub

    rospy.init_node('joint_state_saver')

    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rospy.loginfo("Press Q to move forward in x-axis; Press W to move backward in x-axis; A to move forward in y-axis; S to move backward in y-axis; Z to move forward in z-axis; X to move backward in z-axis.")

    # Create a keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    rospy.spin()

    # Ensure the listener is properly stopped
    listener.stop()

    
if __name__ == '__main__':
    main()
