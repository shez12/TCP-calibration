import rospy
from sensor_msgs.msg import JointState
from pynput import keyboard
import UR10IK as ik

# Global variables to store the latest joint state and positions
current_joint_state = None
joint_positions = None
TCP = [0, 0, 0]

# Set the movement step size (can be adjusted to control speed)
movement_step_size = 0.001

# Set the rate (frequency) of publishing joint state messages
publish_rate = 10  # 10 Hz

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

    # Check if the difference between current and calculated joint positions exceeds 10 degrees (0.1745 radians)
    for i, (current, calculated) in enumerate(zip(joint_positions, q)):
        if abs(current - calculated) > 0.1745:
            rospy.logerr(f"Joint {i} movement exceeds 10 degrees limit. Current: {current}, Calculated: {calculated}")
            return

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

    # Set the rate of publishing messages
    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():
        rate.sleep()

    # Ensure the listener is properly stopped
    listener.stop()

if __name__ == '__main__':
    main()
