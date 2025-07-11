import cv2
import numpy as np
import json
from threading import Thread, Lock
import time
import pyrealsense2 as rs
from spatialmath import SE3

from utils.aruco_util import set_aruco_dict, detect_aruco, estimate_markers_poses
from utils.pose_util import pose_to_SE3
from utils.camera_intrinsics import intrinsics
from utils.kalmen_filter import KalmenFilter

def pose_from_three_points(p1, p2, p3) ->SE3:
    """
    根据三个三维点计算坐标系姿态：
    - x轴：p1指向p2的方向
    - y轴：在三点平面内与x轴正交
    - z轴：垂直于三点平面
    x:p1->p2, y:p1->(p3 direction), z:right hand, translation: p1
    """
    # 转换为numpy数组
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    
    # 计算x轴方向(从p1指向p2)
    x_axis = p2 - p1
    x_axis = x_axis / np.linalg.norm(x_axis)
    
    # 计算平面法向量(即z轴方向)
    v1 = p2 - p1
    v2 = p3 - p1
    z_axis = np.cross(v1, v2)
    z_axis = z_axis / np.linalg.norm(z_axis)
    
    # 计算y轴(在平面内与x轴正交)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    
    # 构造旋转矩阵
    R = np.column_stack((x_axis, y_axis, z_axis))
    
    # 创建SE3位姿(位置在p1，方向由R决定)
    pose = SE3.Rt(R, p1)
    
    return pose

class CameraD435:
    def __init__(self, depth=False, debug=False):
        '''
        init camera, format:bgr8
        depth: get depth info, default=False
        '''
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.depth = depth
        self.debug = debug

        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if self.depth:
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            # 创建对齐对象与color流对齐
            align_to = rs.stream.color  # align_to 是计划对齐深度帧的流类型
            self.align = rs.align(align_to)  # rs.align 执行深度帧与其他帧的对齐

    def start(self):
        if not self.debug:
            self.pipeline.start(self.config)

    def get_frame(self):
        if not self.depth:
            if self.debug:
                img = cv2.imread("imgs/b.jpeg")
                return cv2.resize(img, (640,480))
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            return color_image
        else:
            if self.debug:
                img = cv2.imread("imgs/b.jpeg")
                return 0, 0, cv2.resize(img, (640,480)), 0, 0
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐
 
            aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
            aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧
        
            #### 获取相机参数 ####
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
            color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
        
            #### 将images转为numpy arrays ####
            img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB图
            img_depth = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
        
            return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame
        
    def get_coordinate_with_depth_frame(pixel, aligned_depth_frame, depth_intrin):
        """
        pixel: [x,y]
        """
        dis = aligned_depth_frame.get_distance(pixel[0], pixel[1])  # 获取该像素点对应的深度
        camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, pixel, dis)
        return camera_coordinate
    
    def get_coordinate(self, pixel):
        """
        return 1*4 [x,y,z,1], img_color
        """
        _, depth_intrin, img_color, _, aligned_depth_frame = self.get_frame()
        if self.debug:
            return [0,0,0], img_color
        camera_coordinate = CameraD435.get_coordinate_with_depth_frame(pixel, aligned_depth_frame, depth_intrin)
        return camera_coordinate, img_color

    def stop(self):
        self.pipeline.stop()

class MarkerTracker:
    def __init__(self, camera: CameraD435, marker_json_path: str, se3_ee_camera: SE3):
        '''
        camera: CameraD435
        maker_jason_path: path of maker file   
        T_ee_camera: 4*4 transform matrix from end effector to camera (eye in hand)
        '''
        self.camera = camera
        self.se3_ee_camera = se3_ee_camera
        with open(marker_json_path, 'r') as f:
            self.marker_info = json.load(f)
        
        self.id_list = list(self.marker_info.keys())
        self.position_map = {str(id): None for id in self.id_list}
        self.filter_list = {str(id): KalmenFilter() for id in self.id_list}

        # Thread communication
        self.running = True
        self.lock = Lock()
        self._thread = None
        
        self.image_queues = None
        self._show_images = False

    def start(self, show_images=False):
        """Start tracking in separate thread"""
        self._show_images = show_images
        self.camera.start()
        if self._thread is None:
            self._thread = Thread(target=self._process_data)
            self._thread.daemon = True
            self._thread.start()

    def get_image(self):
        return self.image_queues
    
    def filt_pose(self, marker_id: str, marker_pose):
        """
        filt marker pose
        """
        self.filter_list[marker_id].new_markerpose(marker_pose)
        self.filter_list[marker_id].Kalman_Filter()
        return self.filter_list[marker_id].get_pose()

    def _process_data(self):
        """Process data in separate thread"""
        while self.running:
            img = self.camera.get_frame()
            img_copy = img.copy()

            poses = {str(id): None for id in self.id_list}
            
            # find markers
            corners, ids = detect_aruco(img_copy, draw_flag=self._show_images, aruco_dict=set_aruco_dict(self.marker_info['0']["aruco_dict"]))
            if ids is not None and len(ids)>0:
                # calculate markers poses
                for k, marker_id in enumerate(ids):
                    marker_pose = estimate_markers_poses(corners, marker_size=self.marker_info[str(marker_id)]["marker_size"], intrinsics=intrinsics, frame=img_copy)
                    marker_pose = pose_to_SE3(marker_pose[k])
                    marker_pose = self.se3_ee_camera * marker_pose
                    poses[str(marker_id)] = self.filt_pose(str(marker_id), marker_pose)

            with self.lock:
                self.position_map = poses
                self.image_queues = img_copy

    def show_image(self,marker_id=None,goal_corner=None):
        """
        显示指定相机的图像，返回按键值
        在主线程中调用此方法
        """
        if not self._show_images:
            return None
        img = None
        with self.lock:
            img = self.get_image()
        if img is not None:
            # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # draw corner line
            cv2.imshow('camera', img)
            return cv2.waitKey(1)
        return None

    def stop(self):
        """Stop tracking"""
        self.running = False
        self.camera.stop()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        if self._show_images:
            cv2.destroyAllWindows()

    def get_marker_position(self, marker_id:str):
        """
        Get marker position thread-safely
        return SE3 end effector to marker
        """
        with self.lock:
            if marker_id is None:
                return None
            return self.position_map[marker_id]

    def get_all_positions(self):
        """Get all positions thread-safely"""
        with self.lock:
            return self.position_map.copy()




if __name__ == "__main__":
    camera = CameraD435()
    se3ec = SE3([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1]])
    tracker = MarkerTracker(camera,"marker_info.json", se3ec)
    tracker.start(show_images=True)
    time.sleep(2)
    
    try:
        while True:
            # 显示所有相机图像
            key1 = tracker.show_image()
            
            # 检查退出条件
            if key1 == ord('q'):
                break
            
            # 获取位置数据
            positions = tracker.get_all_positions()
            print("Current positions:", positions)
            # print("\rCurrent positions:", positions, end='')
            
    except KeyboardInterrupt:
        print("\nStopping tracker...")
    finally:
        tracker.stop()

