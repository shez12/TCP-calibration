# TCP-calibration
This is a python code repository for UR10 TCP calibration

--------------------------------------------------------
## Calibration_utils.py
This project implements a Tool Center Point (TCP) calibration method for a UR10 robot. The goal is to optimize the TCP position based on the robot's joint angles and minimize the error through particle swarm optimization (PSO). The code includes forward kinematics calculations, error metric computation, PSO for optimization, and various visualization techniques. One example is given trial1.py.

#### init()
    Should be given at least six set of joint angle list.


#### vectorchoose()

    Selects three points (joint angles) randomly to find a normal vector.


#### pso()

    Performs globalBestPSO to minimize the error metric.
    Important parameters:
                        swarm_size: number of particles
                        dim: dimension of equation
                        max_iters : number of iteriation
                        lower/upper bound : rough bound of tcp



#### verify()
    Verifies the solution by ensuring the average distance to the plane is below a threshold.
    Theshold can be modfied to fit requirements.
    
#### run()

    Runs the PSO optimization and verifies the solution.

### ...

--------------------------------------------------------
## control_UR10.py
This file provides a script to manually control a UR10 robot arm using keyboard inputs. The robot can be moved in Cartesian directions, and joint positions can be saved for later analysis or use. The script uses ROS (Robot Operating System) along with MoveIt! for controlling the robot and performing inverse kinematics. Data would be saved in joint_states.txt.

--------------------------------------------------------
## run_TCP

This function reads joint positions from a file, initializes the tcp_cali class with the joint positions, and runs the calibration process.



