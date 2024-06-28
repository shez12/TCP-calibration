import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np


## UR10 DH Parameters
L1 = rtb.DHLink(d=0.1273,  a=0,     alpha=np.pi/2, offset=0)
L2 = rtb.DHLink(d=0,       a=-0.612, alpha=0,     offset=0)
L3 = rtb.DHLink(d=0,       a=-0.5723, alpha=0,    offset=0)
L4 = rtb.DHLink(d=0.163941, a=0,    alpha=np.pi/2, offset=0)
L5 = rtb.DHLink(d=0.1157,   a=0,    alpha=-np.pi/2, offset=0)
L6 = rtb.DHLink(d=0.0922,  a=0,    alpha=0,     offset=0)

## Joint limits
L1.qlim = [-2*np.pi, 2*np.pi]
L2.qlim = [-2*np.pi, 2*np.pi]
L3.qlim = [-2*np.pi, 2*np.pi]
L4.qlim = [-2*np.pi, 2*np.pi]
L5.qlim = [-2*np.pi, 2*np.pi]
L6.qlim = [-2*np.pi, 2*np.pi]

robot = rtb.DHRobot([L1, L2, L3, L4, L5, L6], name='UR10')

def UR10_IK(q0, T,tool):
    '''
    args:
    q0: list, initial guess of joint angles
    T: SE3, the desired end-effector pose
    tool: list, the tool frame

    return:
    q: list, the joint angles
    
    '''

    robot.tool = tool
    q = robot.ikine_LM(T, q0).q

    for num, i in enumerate(q):
        while (q[num] - q0[num]) > np.pi:
            q[num] -= 2*np.pi
        while (q[num] - q0[num]) < -np.pi:
            q[num] += 2*np.pi
    return q
    
def UR10_FK(q,tool=[0,0,0]):
    '''
    args:
    q: list, joint angles
    
    return:
    T: SE3, the end-effector pose
    
    '''

    robot.tool = tool
    T = robot.fkine(q)
    return T.A
