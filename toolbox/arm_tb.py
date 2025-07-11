import time
from pynput import keyboard
import numpy as np
import os
from spatialmath import SE3
from spatialmath.base import q2r
from Robotic_Arm.rm_robot_interface import *


class ArmController:
    def __init__(self, arm: RoboticArm, handle: rm_robot_handle, ee_tcp:SE3=None):
        """
        ee_tcp: t_ee_tcp, defaut:None(单位矩阵), ee: end effector(Arm Tip), tcp: tool center point
        """
        self.eps_t = 0.001
        self.eps_ryp = 0.005
        if ee_tcp is not None:
            self._ee_tcp:SE3 = ee_tcp
        else:
            self._ee_tcp:SE3 = SE3()
        self._tcp_ee:SE3 = self._ee_tcp.inv()

        self.arm = arm
        self.handle = handle
        
        print("Arm ID: ", self.handle.id)

        self.current_tool_frame = arm.rm_get_current_tool_frame()[1]["name"]
        if(handle.id == -1):
            print("WARNING: cannot find arm")
    
    def __del__(self):
        self.arm.rm_set_arm_stop()
        self.arm.rm_delete_robot_arm()

    def set_sim(self, flag:bool):
        """
        set simulation mode
        return: arm run mode, 0:sim, 1:real, None:fail
        """
        if flag:
            set_mode = 0
        else:
            set_mode = 1
        set_res = self.arm.rm_set_arm_run_mode(set_mode)
        (get_res, mode) = self.arm.rm_get_arm_run_mode()
        if set_res==0 and get_res == 0:
            print(f'arm run mode: {mode} (0:sim, 1:real)')
            return mode
        else:
            print(f'warning: set arm run mode failed, set code:{set_res}, get code:{get_res}')
            return
        
    def change_work_frame(self, name: str):
        """
        change work frame
        name: work frame name ('Base': base frame)
        """
        return self.arm.rm_change_work_frame(name)
    
    def change_tool_frame(self, name: str):
        """
        change tool frame
        name: work frame name ('Arm_Tip': arm tip frame(end effector))
        """
        r = self.arm.rm_change_tool_frame(name)
        self.current_tool_frame = arm.rm_get_current_tool_frame()[1]["name"]
        return r
    
    def update_ee_tcp(self,ee_tcp:SE3):
        self._ee_tcp = ee_tcp
        self._tcp_ee = ee_tcp.inv()

    def get_ee_tcp(self)->SE3:
        return self._ee_tcp
        
    def get_tcp_ee(self)->SE3:
        return self._tcp_ee

    def get_pose(self):
        '''
        get arm current pose from base to tcp (tool point center)
        return: SE3
        '''
        se3_pose: SE3
        (res, state) = self.arm.rm_get_current_arm_state()
        if res != 0:
            print(f'warning: get state failed, code:{res}')
            return
        pose = state['pose']
        se3_pose = self.pose2se3(pose)
        return se3_pose
    
    def get_joint(self):
        '''
        get arm current joint
        return: SE3
        '''
        (res, state) = self.arm.rm_get_current_arm_state()
        if res != 0:
            print(f'warning: get state failed, code:{res}')
            return
        joint = state['joint']
        return joint

    def move_p(self, pose: SE3, v:int=5, r:int=0, connect:int=0, block:int=1)->int:
        """
        move arm to target pose

        pose: target pose SE3 : T_base_tcp
        v: speed percent: 1~100
        r: 交融半径百分比系数: 0~100。
        block: 0: 非阻塞模式，发送指令后立即返回。 1: 阻塞模式，等待机械臂到达目标位置或规划失败后才返回。

        return: 0:success, realman error code
        """
        # send command
        _pose = self.se32pose(pose)
        result = self.arm.rm_movej_p(_pose, v, r, connect, block)
        if (result!=0):
            print(f"WARNING: move p failed. code:{result}")
        return result
        
    def move_xyz(self, xyz: list, v:int=5, block:int=1)->int:
        """
        move arm to target xyz with current poseture

        xyz: [x, y, z]
        v: speed percent: 1~100
        block: 0: 非阻塞模式，发送指令后立即返回。 1: 阻塞模式，等待机械臂到达目标位置或规划失败后才返回。

        return: 0:success, realman error code
        """
        # send command
        pose = self.get_pose()
        pose.t = xyz
        result = self.move_p(pose, v, block=block)
        if (result!=0):
            print(f"WARNING: move p failed. code:{result}")
        return result
    
    def move_tool_p(self, pose: SE3, v:int=5, r:int=0, connect:int=0, block:int=1)->int:
        """
        move tool

        pose: move tool
        v: speed percent: 1~100
        r: 交融半径百分比系数: 0~100。
        block: 0: 非阻塞模式，发送指令后立即返回。 1: 阻塞模式，等待机械臂到达目标位置或规划失败后才返回。

        return: 0:success, realman error code
        """
        # send command
        current_pose = self.get_pose()
        return self.move_p(current_pose*pose, v, r, connect, block)
    
    def move_step_x(self, x, v:int=5, block:int=1)->int:
        """
        move step with respect to base coordinate

        x: x distance in base frame (mm)
        v: speed percent: 1~100
        block: 0: 非阻塞模式，发送指令后立即返回。 1: 阻塞模式，等待机械臂到达目标位置或规划失败后才返回。

        return: 0:success, realman error code
        """
        return self.arm.rm_set_pos_step(rm_pos_teach_type_e.RM_X_DIR_E, x, v, block)
    
    def move_step_y(self, y, v:int=5, block:int=1)->int:
        """
        move step with respect to base coordinate

        y: y distance in base frame (mm)
        v: speed percent: 1~100
        block: 0: 非阻塞模式，发送指令后立即返回。 1: 阻塞模式，等待机械臂到达目标位置或规划失败后才返回。

        return: 0:success, realman error code
        """
        return self.arm.rm_set_pos_step(rm_pos_teach_type_e.RM_Y_DIR_E, y, v, block)
    
    def move_step_z(self, z, v:int=5, block:int=1)->int:
        """
        move step with respect to base coordinate

        z: z distance in base frame (mm)
        v: speed percent: 1~100
        block: 0: 非阻塞模式，发送指令后立即返回。 1: 阻塞模式，等待机械臂到达目标位置或规划失败后才返回。

        return: 0:success, realman error code
        """
        return self.arm.rm_set_pos_step(rm_pos_teach_type_e.RM_Z_DIR_E, z, v, block)

    def move_follow(self, pose:SE3):
        """
        pose: target pose SE3 : T_base_tcp
        return: 0:success, 1: 控制器返回false, 参数错误或机械臂状态发生错误。1: 数据发送失败，通信过程中出现问题。
        """
        _pose = self.se32pose(pose)
        result = self.arm.rm_movep_follow(_pose)
        if (result!=0):
            print(f"WARNING: move p failed. code:{result}")
        return result
    
    def teach_trajectory(self, frequency=100, file_path=None)->list[SE3]:
        """
        frequency: frequency of the trajectory
        return list of poses with SE3: T_base_ee
        """
        print('press e to end teach')
        def on_press(key):
            if key == keyboard.KeyCode.from_char('e'):
                print("teach ended")
                return False  # 停止监听

        listener = keyboard.Listener(on_press=on_press)
        listener.start()

        trajectory = []
        first_idx_stop = False
        first_idx = 0
        last_idx = 0
        while True:
            pose = self.get_pose()
            trajectory.append(pose)
            if len(trajectory) > 1:
                if not first_idx_stop:
                    diff = trajectory[-1].inv()*trajectory[-2]
                    diff_t = np.abs(diff.t).sum()
                    diff_rpy = np.abs(diff.rpy()).sum()
                    if diff_t > self.eps_t or diff_rpy > self.eps_ryp:
                        last_idx=len(trajectory)
                        first_idx_stop = True
                    else:
                        first_idx = len(trajectory)
                else:
                    diff_last = trajectory[-1].inv()*trajectory[last_idx]
                    if np.abs(diff_last.t).sum() > self.eps_t or np.abs(diff_last.rpy()).sum() > self.eps_ryp:
                        last_idx = len(trajectory)
            # print(pose)
            time.sleep(1/frequency)
            if not listener.is_alive():  # 如果监听器停止
                break
        
        if (first_idx<last_idx):
            trajectory = trajectory[first_idx:last_idx]
            # save
            if file_path is not None:
                ArmController.save_se3_list(file_path, trajectory)
            
            return trajectory
        else:
            print("Warning: unvalid trajectory")
            return None
    
    def move_trajectory(self, trajectory:list[SE3]=None, file_path:str=None, v:int=5, r:int=0):
        assert trajectory is None or file_path is None
        if file_path is not None:
            trajectory=ArmController.load_se3_list(file_path)
        assert trajectory is not None

        for pose in trajectory:
            # print(pose)
            # result = self.move_p(pose,v, r,0,1)
            result = self.move_follow(pose)
            # time.sleep(0.1)
            if result!=0:
                print(f"warning: move trajectory error code: {result}")

    def se32pose(self, se3:SE3, flag=1):
        """
        se3: T_base_tcp
        flag (int, optional): 选择姿态表示方式，默认欧拉角表示姿态
            - 0: 返回使用四元数表示姿态的位姿列表[x,y,z,w,x,y,z]
            - 1: 返回使用欧拉角表示姿态的位姿列表[x,y,z,rx,ry,rz]
        """
        if self.current_tool_frame == 'Arm_Tip':
            se3=se3*self._tcp_ee
        mat = rm_matrix_t(4,4,se3.A)
        return self.arm.rm_algo_matrix2pos(mat,flag=flag)
    
    def pose2se3(self, pose:list, flag=1):
        """
        flag (int, optional): 选择姿态表示方式，默认欧拉角表示姿态
            - 0: 返回使用四元数表示姿态的位姿列表[x,y,z,w,x,y,z]
            - 1: 返回使用欧拉角表示姿态的位姿列表[x,y,z,rx,ry,rz]
        return: T_base_tcp
        """
        assert flag==0 or flag==1
        if flag==1:
            assert len(pose)==6
            e = pose[3:]
        elif flag==0:
            assert len(pose)==7
            e = self.arm.rm_algo_quaternion2euler(pose[3:])
        out = SE3.Trans(pose[:3]) * SE3.RPY(e,order='zyx')
        if self.current_tool_frame == 'Arm_Tip':
            return out*self._ee_tcp
        return out
    
    def mat2se3(self, mat, t=None):
        """
        array mat to SE3
        mat: T_base_tcp 3*3 or 4*4 np array
        t: 3*1 translation
        return SE3
        """
        T = np.eye(4)
        if mat.shape[0]==3:
            T[:3,:3] = mat
            if t is not None:
                T[:3,3]=t
        else:
            T=mat
        T = T@self._tcp_ee.A
        pose =  self.arm.rm_algo_matrix2pos(rm_matrix_t(4,4,T))
        se3_pose = SE3.Trans(pose[:3]) * SE3.RPY(pose[3:],order='zyx')
        if self.current_tool_frame == 'Arm_Tip':
            return se3_pose*self._ee_tcp
        return se3_pose
    
    def p_transform(T: SE3, point):
        """ 
        position transform
        T: T_frame1_frame2
        point: point coordinate in frame2
        return: point coordinate in frame1
        """
        out = SE3.Trans(point)
        out = T*out
        return out.t
    
    def vf_transform(T: SE3, point):
        """ 
        velocity/force transform
        T: T_frame1_frame2
        point: point coordinate in frame2
        return: point coordinate in frame1
        """
        out = np.array(point)
        out = T.R@out.reshape((3,1))
        out = out.reshape((3,)).tolist()
        return out

    def save_se3_list(file_path:str, se3_list:list[SE3]):
        """
        save se3 list to .npy file
        file_path should end with .npy
        """
        if file_path is not None:
            directory = os.path.dirname(file_path)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
        
        _, ext = os.path.splitext(file_path)
        assert ext.lower() == '.npy'
        np.save(file_path, np.array(se3_list))

    def load_se3_list(file_path:str) ->list[SE3]:
        try:
            loaded_data = np.load(file_path)
            assert loaded_data.shape[1:]==(4,4)
            se3_list = [SE3.Rt(matrix[:3,:3], matrix[:3,3]) for matrix in loaded_data]
            if len(se3_list) != loaded_data.shape[0]:
                print("WARNING: load se3 list may not be complete")
            return se3_list
        except:
            print("FAILED: load se3 list file failed")


if __name__ == "__main__":
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
    pose1 = SE3.Rz(-90,unit="deg")
    # controller = ArmController(arm, handle,pose1)
    controller = ArmController(arm, handle)
    # if controller.set_sim(True) != 0:
    if controller.set_sim(False) != 1:
        exit()
    
    # current_pose = controller.get_pose()
    # if current_pose is not None:
    #     print(f'a: {controller.move_p(current_pose*SE3.Ry(-30,unit="deg"))}')
    # time.sleep(5)

    # ArmController.save_se3_list("se.npy", [SE3.Eul(1,1.5,0)])
    # T = ArmController.load_se3_list("se.npy")

    # ArmController.save_se3_list("se.npy", [SE3.Eul(1,1.5,0)])
    # T = ArmController.load_se3_list("se.npy")
