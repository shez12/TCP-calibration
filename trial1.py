import numpy as np
from calibration_utils import OMOPSO
import matplotlib.pyplot as plt
import UR10Fk as fk

# points = [
#         [273.41,-121.61,-96.71,-28.52,107.21,188.94],
#         [273.36,-121.84,-96.67,-31.06,109.41,188.95],
#         [273.33,-121.78,-96.72,-32.66,109.56,188.95],
#         [273.35,-123.04,96.04,-25.73,110.99,188.94],
#         [273.38,-123.59,-95.99,-25.44,112.73,188.95],
#         [273.38,-123.94,-95.7,-22.6,112.53,188.94]
# ]

points = [
         [312.97,-127.67,-116.68,1.65,39.17,187.07],
        [318.69,-129.73,-130.53,23.72,31.06,187.06],
        [303.96,-124.05,-111.19,-7.57,53.16,187.06],
        [275.14,-120.23,-114.45,2.04,102.37,187.07],
        [261.87,-129.41,-112.99,10.63,131.26,187.07],
        [308.47,-127.20,-123.43,15.71,38.45,187.06]


]


'''     
        [310.96,-121.61,-125.31,12.14,48.5,187.06],
          

'''

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Plot the points
points_array = []
for joint_angle in points:
        fk_t = fk.HTrans(joint_angle)
        points_array.append(fk_t[0:3, 3])

points_array = np.array([np.array(point).flatten() for point in points_array])
# Extract X, Y, Z coordinates
X = points_array[:, 0]
Y = points_array[:, 1]
Z = points_array[:, 2]

# print("point coordinate: " ,points_array)
ax.scatter(X, Y, Z, color='r', label='Points')
plt.show()

'''
find solution:  [[-0.07787012]
 [-0.35474251]
 [-0.08979831]]
current best average dist:  1.686715923615746e-05
current max dist:  5.060147770858446e-05
current min dist:  3.382514572677121e-07

'''

test_joint_angles_radians = [np.array(angles) * np.pi / 180 for angles in points] # 输出结果 test_joint_angles_radians
data = OMOPSO(test_joint_angles_radians)
data.run()
