import numpy as np
import UR10Fk as fk  
import pyswarms as ps
import random
import matplotlib.pyplot as plt

class tcp_cali:
    """
    Class for analyzing robot joint angles and calculating error based on Tool Center Point (TCP).
    """

    def __init__(self, joint_angle_list):
        """
        Initializes the class with a list of joint angles.

        Args:
            joint_angle_list: List of joint angles in radians, where each list element represents a robot configuration.
                                size>=6
        """
        self.joint_angle_list = joint_angle_list
        self.choose = self.vectorchoose() # choose six points randomly; 3 for plane vector, 3 for dot product

    def vectorchoose(self):
        '''
        Choose three points(joint angles) to find a normal vector.
        
        
        '''
        if len(self.joint_angle_list)<6:
            raise KeyError("joint angle list is too short")
        
        if len(self.joint_angle_list)<10:
            print("recommmend to collect 10 points to get a better solution")

        randomPlane=[]
        while (len(randomPlane)<6):# choose n points, n can be changed
            random_num = random.randint(0,len(self.joint_angle_list)-1)
            if random_num not in randomPlane:
                randomPlane.append(random_num)
        
        print("now choose",randomPlane)
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
            
            vector_list = []# vector for plane
            # Calculate forward kinematics for current joint angles
            fk_t = fk.HTrans(self.joint_angle_list[vector_choose[0]])
            fk_r1 = fk_t[0:3, 0:3]  # Rotation matrix
            fk_p1 = fk_t[0:3, 3]  # Position vector
            for i in vector_choose[1:3]:
                fk_t2 = fk.HTrans(self.joint_angle_list[i])
                fk_r2 = fk_t2[0:3, 0:3]
                fk_p2 = fk_t2[0:3, 3]
                cross_vector = fk_r2 @ tcp - fk_r1 @ tcp + fk_p2 - fk_p1
                cross_vector = cross_vector / np.linalg.norm(cross_vector)
                vector_list.append(cross_vector)
           
            vector_list2 = []
            for i in vector_choose[3:]:
                    fk_t3 = fk.HTrans(self.joint_angle_list[i])
                    fk_r3 = fk_t3[0:3, 0:3]
                    fk_p3 = fk_t3[0:3, 3]
                    dot_vector = fk_r3 @ tcp - fk_r1 @ tcp + fk_p3 - fk_p1
                    dot_vector = dot_vector / np.linalg.norm(dot_vector)
                    vector_list2.append(dot_vector)

            # calculate planeVector
            planeVector = np.cross(self.mat2array(vector_list[0]), self.mat2array(vector_list[1]))
            planeVector = planeVector / np.linalg.norm(planeVector)
            equation = 0
            # calculate the error
            for i in vector_list2:
                equation+=(abs(np.dot(self.mat2array(planeVector), self.mat2array(i))))
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

    def pso(self):
        """
        Performs globalBestPSO to minimize the error metric.
        Returns:
            tuple: A tuple containing the minimum cost and corresponding joint variables.
        """
        def line_decrease(iteration,max_iters):
            return 0.9-0.5*(iteration/max_iters)
        swarm_size = 200
        dim = 3
        options = {"c1": 2, "c2": 2.5}
        lower_bound = np.array([-0.1,-0.3,0.2])
        upper_bound = np.array([0.1,0,0.5])

        constraints = (lower_bound, upper_bound)
        max_iters = 500
        optimizer = ps.single.GlobalBestPSO(
            n_particles=swarm_size,
            dimensions=dim,
            options=options,
            bounds=constraints,
        )

        #Perform optimization
        for iter in range(max_iters):
            optimizer.options["w"] = line_decrease(iter,max_iters)
            cost, tcp = optimizer.optimize(self.func, iters=1, verbose=False)
            print("w is",line_decrease(iter,max_iters),"iter is", iter,"cost is",cost)

        self.tcp= np.array(tcp).reshape(3, 1)
        print("current solution :",self.tcp,"current cost",cost)
        return cost, tcp


    def dist(self):
        '''
        Find the distance from each point to the plane.
        
        '''
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
        '''
        plot the plane and points.
        Args:
            X,Y,Z: point coordinate
            C: plane coefficient
        
        '''
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
        ax.plot_surface(X_plane, Y_plane, Z_plane, color='r', alpha=0.5, label='Fitted Plane')
    
        # Set labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
    
        plt.show()

    


    def verify(self):
        '''
        verify the solution.
        '''
        threshold = 0.0001
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
            self.pso()
            dist_list = self.dist()

        print("find solution: ", self.tcp)
        print("current best average dist: " , np.mean(dist_list))
        print("current max dist: " , max(dist_list))
        print("current min dist: " , min(dist_list))
        return self.tcp,self.dist()



    def plot_info(self,data):
        '''

        plot the statistical information of the data.

        '''


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

        self.pso()
        # print(self.dist())
        self.verify()



