import rospy
from sensor_msgs.msg import JointState
from pynput import keyboard

# 全局变量用于存储最新的关节状态
current_joint_state = None

def joint_state_callback(msg):
    global current_joint_state
    current_joint_state = msg

def on_press(key):
    if key == keyboard.Key.esc:
        # 停止监听
        return False
    else:
        save_joint_state()

def save_joint_state():
    if current_joint_state is not None:
        joint_names = current_joint_state.name
        joint_positions = current_joint_state.position

        with open('joint_states.txt', 'a') as file:
            file.write("Joint Names: {}\n".format(joint_names))
            file.write("Joint Positions: {}\n\n".format(joint_positions))
        
        rospy.loginfo("Joint state saved!")

def main():
    global current_joint_state

    rospy.init_node('joint_state_saver')

    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    rospy.loginfo("Press any key to save the current joint state. Press ESC to exit.")

    # 创建一个键盘监听器
    with keyboard.Listener(on_press=on_press) as listener:
        rospy.spin()
        listener.join()

if __name__ == '__main__':
    main()
