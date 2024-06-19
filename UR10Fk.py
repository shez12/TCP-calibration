## UR5/UR10 Inverse Kinematics - Ryan Keating Johns Hopkins University


# ***** lib
import numpy as np

from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi



global mat
mat=np.matrix


# ****** DH Table ******

global d, a, alph


d = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])#ur10 m

a =mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])#ur10    m

alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ]) # ur10




# ************************************************** FORWARD KINEMATICS

def AH( n,th  ):
  '''
  args:
  n: int, the number of the joint
  th: list, the joint angles(rad)

  '''

  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]

  Rzt = mat([[cos(th[n-1]), -sin(th[n-1]), 0 ,0],
	         [sin(th[n-1]),  cos(th[n-1]), 0, 0],
	         [0,               0,              1, 0],
	         [0,               0,              0, 1]],copy=False)
      

  Rxa = mat([[1, 0,                 0,                  0],
			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
			 [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
	    

  return A_i

def HTrans(th): 
  '''
  args:
  th: list, the joint angles(rad)
  '''

  A_1=AH( 1,th )
  A_2=AH( 2,th )
  A_3=AH( 3,th )
  A_4=AH( 4,th)
  A_5=AH( 5,th)
  A_6=AH( 6,th)
      
  T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_06


 
 