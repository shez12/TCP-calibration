

import numpy as np
import UR10Fk as fk  # Assuming UR10Fk is a custom module for forward kinematics
import pyswarms as ps
import random
import matplotlib.pyplot as plt

class OMOPSO:
    """
    Class for analyzing robot joint angles and calculating error based on Tool Center Point (TCP).
    """

    def __init__(self, joint_angle_list):
        """
        Initializes the class with a list of joint angles.

        Args:
            joint_angle_list: List of joint angles in radians, where each list element represents a robot configuration.
        """
        self.joint_angle_list = joint_angle_list
        self.choose = self.vectorchoose()


    def vectorchoose(self):
        randomPlane=[]
        while (len(randomPlane)<3):
            random_num = random.randint(0,len(self.joint_angle_list)-1)
            randomPlane.append(random_num)
            randomPlane = list(np.unique(randomPlane))
        return randomPlane

    


    def func(self, tcpList):
        """
        Calculates error metric based on TCP and robot kinematics.

        Args:
            tcp: A 3x1 NumPy array representing the Tool Center Point (TCP).

        Returns:
            List of float: The calculated error metric.

        """

        vector_choose = self.choose


        equationList=[]
        for tcp in tcpList:
            tcp = np.array(tcp).reshape(3, 1)
            vector_list = []
            # Calculate forward kinematics for current joint angles
            fk_t = fk.HTrans(self.joint_angle_list[0])
            fk_r1 = fk_t[0:3, 0:3]  # Rotation matrix
            fk_p1 = fk_t[0:3, 3]  # Position vector
            for i in vector_choose[1:]:
                fk_t2 = fk.HTrans(self.joint_angle_list[i])
                fk_r2 = fk_t2[0:3, 0:3]
                fk_p2 = fk_t2[0:3, 3]
                vector_list.append(fk_r2 @ tcp - fk_r1 @ tcp + fk_p2 - fk_p1)
           
            vector_list2 = []
            for i in range(len(self.joint_angle_list)):
                if (i not in vector_choose):
                    fk_t3 = fk.HTrans(self.joint_angle_list[i])
                    fk_r3 = fk_t3[0:3, 0:3]
                    fk_p3 = fk_t3[0:3, 3]
                    vector_list2.append(fk_r3 @  tcp - fk_r1 @ tcp + fk_p3 - fk_p1)

            # calculate planeVector
            planeVector = np.cross(self.mat2array(vector_list[0]), self.mat2array(vector_list[1]))
            equation = 0
            for i in range(len(vector_list2)):
                equation+=(abs(np.dot(self.mat2array(planeVector), self.mat2array(vector_list2[i]))))
            equationList.append(equation)
        return equationList

    @staticmethod
    def mat2array(mat):
        """
        Converts a matrix to a flattened NumPy array.

        Args:
            mat: A NumPy matrix.

        Returns:
            NumPy array: The flattened version of the input matrix.
        """

        return np.asarray(mat).flatten()

    def omopso(self):
        """
        Performs PSO (Particle Swarm Optimization) to minimize the error metric.

        Args:
            lower_bound: A 3x1 NumPy array representing the lower bounds for TCP coordinates.
            upper_bound: A 3x1 NumPy array representing the upper bounds for TCP coordinates.

        Returns:
            tuple: A tuple containing the minimum cost and corresponding joint variables.
        """
        def line_decrease(iteration,max_iters):
            return 0.9-0.6*(iteration/max_iters)

        print("solving equations")
        swarm_size = 200
        dim = 3
        options = {"c1": 2, "c2": 2.5}
        lower_bound = np.array([-0.1,-0.3,0.2])
        upper_bound = np.array([0.1,0,0.5])
        constraints = (lower_bound, upper_bound)
        max_iters = 300
        optimizer = ps.single.GlobalBestPSO(
            n_particles=swarm_size,
            dimensions=dim,
            options=options,
            bounds=constraints,
        )

        #Perform optimization
        for iter in range(max_iters):
            optimizer.options["w"] = line_decrease(iter,max_iters)
            print("w is",line_decrease(iter,max_iters),"iter is", iter)
            cost, tcp = optimizer.optimize(self.func, iters=1, verbose=False)


        self.tcp= np.array(tcp).reshape(3, 1)
        print("current solution :",self.tcp)
        return cost, tcp


    def dist(self):
        #find each points coordinate
        points = []
        for joint_angle in self.joint_angle_list:
            fk_t = fk.HTrans(joint_angle)
            fk_p = fk_t[0:3, 3]
            fk_R = fk_t[0:3, 0:3]
            point = fk_R@self.tcp+fk_p
            points.append(point)
        points_array = np.array([np.array(point).flatten() for point in points])
        # Extract X, Y, Z coordinates
        X = points_array[:, 0]
        Y = points_array[:, 1]
        Z = points_array[:, 2]

        A = np.c_[X, Y, np.ones(X.shape[0])]
        B = Z

        # least square method
        C, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
        # Plane coefficients
        a, b, d = C[0], C[1], C[2]
        c = -1
        self.plot_plane(X,Y,Z,[a,b,c,d])
        # Calculate the distance from each point to the plane
        distances = np.abs(a * X + b * Y + c * Z + d) / np.sqrt(a**2 + b**2 + c**2)
        self.plot_info(distances)
        return distances

    
    def plot_plane(self,X,Y,Z,C):
        a,b,c,d = C
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Plot the points
        ax.scatter(X, Y, Z, color='b', label='Points')
    
        # Create a meshgrid for the plane
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        X_plane, Y_plane = np.meshgrid(np.linspace(xlim[0], xlim[1], 10),
                                    np.linspace(ylim[0], ylim[1], 10))
    
        Z_plane = (-a * X_plane - b * Y_plane - d) / c
    
        # Plot the plane
        # ax.plot_surface(X_plane, Y_plane, Z_plane, color='r', alpha=0.5, label='Fitted Plane')
    
        # Set labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
    
        plt.show()

    


    def verify(self):
        
        threshold = 0.002
        dist_list = self.dist()
        n=0
        while np.mean(dist_list)>threshold:
            print("current best average dist: " , np.mean(dist_list))
            print("current max dist: " , max(dist_list))
            print("current min dist: " , min(dist_list))
            if n>8:
                raise KeyError("recollect points")
            print("current solution is not good enough, continue")
            n+=1
            self.choose = self.vectorchoose()
            self.omopso()
            dist_list = self.dist()

        print("find solution: ", self.tcp)
        print("current best average dist: " , np.mean(dist_list))
        print("current max dist: " , max(dist_list))
        print("current min dist: " , min(dist_list))
        return self.tcp,self.dist()



    def plot_info(self,data):
        data = [i*1000 for i in data]
        max_val = np.max(data)
        min_val = np.min(data)
        mean_val = np.mean(data)
        std_val = np.std(data)
        stats = {

            'Max': max_val,

            'Min': min_val,

            'Mean': mean_val,

            'Std Dev': std_val

        }
        fig, ax = plt.subplots()
        bars = ax.bar(stats.keys(), stats.values(), color=['blue', 'orange', 'green', 'red'])
        for bar in bars:

            yval = bar.get_height()

            ax.text(bar.get_x() + bar.get_width()/2, yval, round(yval, 2), va='bottom')  # va: vertical alignment

        ax.set_title('Statistical Measures of Data (unit mm)')

        ax.set_ylabel('Value')

        plt.show()
        




    def run(self):
        # print("now choose",self.plane_point)
        self.omopso()
        # print(self.dist())
        self.verify()



# if __name__ =="__main__":
   
#     test_joint_angles_degrees = [[0,0,4.592-90,3.883,40.894,93.418],
#                          [-16.360,-69.026,3.156-90,3.776,39.137,133.812],    
#                          [-18.966,-72.777,7.720-90,15.494,51.803,170.749],    
#                          [-15.454,-72.356,10.098-90,30.897,27.092,129.078],    
#                          [-17.059,-76.277,13.533-90,30.244,33.174,28.440],    
#                          [-16.637,-72.496,8.437-90,33.510,43.938,50.159]]

#     test_joint_angles_radians = [np.array(angles) * np.pi / 180 for angles in test_joint_angles_degrees] # 输出结果 test_joint_angles_radians
#     data = OMOPSO(test_joint_angles_radians)
#     data.run()


#     pass



