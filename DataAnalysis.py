import numpy as np
import UR10Fk as fk
import omopso
import math
from sympy import symbols,simplify



global mat
mat = np.matrix

class dataAnalysis:
    def __init__(self,jointAngleList):

        '''
        jointangle --> [a1,a2,a3,a4,a5,a6]
        jointanglelist -->[jointangle1,jointangle2,....] len>=6
        '''
        self.listSize = len(jointAngleList)
        self.jointAngleList = jointAngleList
        x, y, z = symbols('x y z')
        self.TCP = mat([x,y,z]).T # 3*1 unknown vertocr

        pass

    def getFk(self,jointAngle):
        FK_T = fk.HTrans(jointAngle)

        FK_R = FK_T[0:3,0:3]  # 3*3 matrix
        FK_P = FK_T[0:3,3]    # 3*1 vertor
        return FK_R,FK_P



    def computeVectors(self,jointList,num):

        '''
        based on joint angle and UR10fk, get vectors between sample points

        v12= (R2e-R1e)Ptcp+p2e-p1e 

        num: number of verctors

        '''
        vectorList = []
        R1e,p1e = self.getFk(jointList[0])    
        for i in range(num-1):
            R2e,p2e = self.getFk(jointList[i+1])
            vectorList.append( 
                R2e*self.TCP-R1e*self.TCP+p2e-p1e
            )

        return vectorList

    def mat2Array(self,mat):
        newArray = np.asarray(mat).flatten()
        return newArray



    def formEquation(self,vector_list):
        '''
        two vector cross product get the plane vector
        all vectors on that plane get 0 while dot product with the plane vector
        '''
        planeVector = np.cross(self.mat2Array(vector_list[0]),self.mat2Array(vector_list[1]))
         
        equation=0
        for i in range(2,len(vector_list)):
            equation+=np.dot(self.mat2Array(planeVector),self.mat2Array(vector_list[2]))
        
        return simplify(equation)
            
    
    def groupEquationList(self):
        '''
        consider six points as a group and form one equation
        
        '''
        equationList = []
        for num in range(math.floor(len(self.jointAngleList)/6)):
            
            if num<0:
                raise KeyError("need at least six group of data")
            vectorList = self.computeVectors(self.jointAngleList[num*6:(num+1)*6],6)
            equationList.append(self.formEquation(vectorList).copy())
            
    
        return equationList


    def omopso(self,equationList):
        '''
        solve cubic equtions using omopso method
        
        '''
        return omopso.omopsoCompute(equationList)




    def bestFitPlane(points):
    #     # Create matrices A and b
    #     A = np.c_[points[:,0], points[:,1], points[:,2], np.ones(points.shape[0])]
    #     b = np.zeros(points.shape[0])
        
    #     # Solve the least squares problem to find the plane parameters
    #     plane_params, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        
    #     # Plane parameters a, b, c, d
    #     a, b, c, d = plane_params
    
        
        # return plane_params, distances
        pass

    def verify(self,Ptcp_e):
        '''
        get Ptcp, using least square method find the their surface anc compute distance between points and surface
        if distance is acceptable, clibration is over
        if not acceptable, regroup points and try again
        if still not acceptable, recollect points 
        '''
        Ptcp_b_list =[]
        for jointAngle in self.jointAngleList:
            Re,pe = self.getFk(jointAngle)
            Ptcp_b_list.append(Re*Ptcp_e+pe)
        # least square
        _,distances = self.bestFitPlane(Ptcp_b_list)
        

        pass




if __name__ =="__main__":
    
    test_joint_angles_degrees = [[0,0,4.592,3.883,40.894,93.418],
                         [-16.360,-69.026,3.156,3.776,39.137,133.812],    
                         [-18.966,-72.777,7.720,15.494,51.803,170.749],    
                         [-15.454,-72.356,10.098,30.897,27.092,129.078],    
                         [-17.059,-76.277,13.533,30.244,33.174,28.440],    
                         [-16.637,-72.496,8.437,33.510,43.938,50.159]] 

    test_joint_angles_radians = [np.array(angles) * np.pi / 180 for angles in test_joint_angles_degrees] # 输出结果 test_joint_angles_radians
    data = dataAnalysis(test_joint_angles_radians)
    print(data.omopso(data.groupEquationList()))



    pass

