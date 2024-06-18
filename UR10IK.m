%%利用标准D-H法建立多轴机器人
clear;
clc;
L1 = Link('d', 0.1273,  'a', 0, 'alpha', pi/2,'offset',0);    %Link 类函数;offset建立初始的偏转角
L2 = Link('d', 0,       'a', -0.612, 'alpha', 0,  'offset',0);
L3 = Link('d', 0,       'a', -0.5723, 'alpha', 0,'offset',0);
L4 = Link('d', 0.163941, 'a', 0, 'alpha', pi/2,'offset',0);
L5 = Link('d', 0.1157,   'a', 0, 'alpha', -pi/2,'offset',0);
TCP = [1,2,3];
L6 = Link('d', 0.0922,  'a', 0, 'alpha', 0,   'offset',0);

L1.qlim = [-pi/2,pi];%利用qlim设置每个关节的旋转角度范围
robot=SerialLink([L1,L2,L3,L4,L5,L6],'name','UR10');   %SerialLink 类函数
robot.tool=([-0.00624,-0.224,0.377]);
%% 普通机器人的示教展示
robot.fkine([0,0,0,0,0,0])
robot.display();%展示出机器人的信息
teach(robot);%调出示教滑块

%% 逆解


mask=[1,1,1,0,0,0];
robot.ikine(robot.fkine([0,0,0,0,0,0]),mask)
