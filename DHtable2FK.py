import numpy as np
PI =np.pi
def DHFK(Q):
    Q = np.deg2rad(Q)
    # DH 参数      [  a, alpha,  d,        theta]
    DH = np.array([ [0,     -PI,     -400,        Q[0]],
                    [180,     PI/2,    0,         Q[1]],
                    [600,     0,       0,         Q[2]-PI/2],
                    [ 40,    PI/2,     -620,      Q[3]],
                    [ 0 ,   -PI/2,     0,         Q[4]],
                    [0,     PI/2,      -80,       Q[5]] ])#kr5
    

    # 机械臂正运动学
    Trans = np.eye(4)
    for i in range(len(DH)):
        Rx_alpha = np.array([[1,               0,                0,0],
                            [0,np.cos(DH[i][1]),-np.sin(DH[i][1]),0],
                            [0,np.sin(DH[i][1]), np.cos(DH[i][1]),0],
                            [0,               0,                0,1]])

        Dx_a = np.array([[1, 0, 0, DH[i][0]],
                        [0, 1, 0,        0],
                        [0, 0, 1,        0],
                        [0, 0, 0,        1]])
        
        Rz_theta = np.array([[np.cos(DH[i][3]),-np.sin(DH[i][3]),0,0],
                            [np.sin(DH[i][3]), np.cos(DH[i][3]),0,0],
                            [               0,                0,1,0],
                            [               0,                0,0,1]])
        
        Dz_d = np.array([[1, 0, 0,        0],
                        [0, 1, 0,        0],
                        [0, 0, 1, DH[i][2]],
                        [0, 0, 0,        1]])
        Trans = Trans @ (Rz_theta@Dz_d@Dx_a@Rx_alpha)

    return Trans

if __name__ == "__main__":
    a =[0,0,0,0,0,0]
    print(DHFK(a))
    tcp = np.array([-5,10,-80.9,1]).reshape(4,1)
    print(DHFK(a)@tcp)