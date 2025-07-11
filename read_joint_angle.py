from pynput import keyboard
from Robotic_Arm.rm_robot_interface import *
from toolbox.arm_tb import ArmController
import time

# 全局变量用于存储最新的关节状态
current_joint_state = None

robo_arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robo_arm.rm_create_robot_arm("192.168.1.18", 8080)
arm = ArmController(robo_arm, handle)

def save_joint_state(joint_state):
    with open('joint_states.txt', 'a') as file:
        file.write("Joint Positions: {}\n\n".format(joint_state))

def on_press(key):
    try:
        if key.char == 's':    # 按下s键保存关节状态
            # joint_state = arm.get_joint()
            # save_joint_state(joint_state)
            print("Joint state saved!")
        elif key.char == 'esc':    # 按下esc键退出程序
            return False  # This will stop the listener
    except AttributeError:
        pass

if __name__ == '__main__':
    # Set up the keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    print("Press 's' to save joint state, 'esc' to exit...")
    
    # Keep the program running until listener stops
    while listener.is_alive():
        time.sleep(0.1)
    
    print("Program ended")