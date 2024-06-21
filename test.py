import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

# 创建链接（Link）
L1 = rtb.DHLink(d=0.1273,  a=0,     alpha=np.pi/2, offset=0)
L2 = rtb.DHLink(d=0,       a=-0.612, alpha=0,     offset=0)
L3 = rtb.DHLink(d=0,       a=-0.5723, alpha=0,    offset=0)
L4 = rtb.DHLink(d=0.163941, a=0,    alpha=np.pi/2, offset=0)
L5 = rtb.DHLink(d=0.1157,   a=0,    alpha=-np.pi/2, offset=0)
L6 = rtb.DHLink(d=0.0922,  a=0,    alpha=0,     offset=0)

# 设置关节角度范围
L1.qlim = [-np.pi/2, np.pi]
L2.qlim = [-np.pi, np.pi]
L3.qlim = [-np.pi, np.pi]
L4.qlim = [-np.pi, np.pi]
L5.qlim = [-np.pi, np.pi]
L6.qlim = [-np.pi, np.pi]

# 创建机器人模型
robot = rtb.DHRobot([L1, L2, L3, L4, L5, L6], name='UR10')
robot.tool = [1,1,1]
# 打印机器人信息
print(robot)
q1 = [0, 0, 0, 0, 0, 0]
T = robot.fkine(q1)
print(T)
print(T.A)
sol = robot.ik_LM(T,q0=q1)
print(sol)    