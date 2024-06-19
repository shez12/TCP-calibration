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
robot.tool=([-0.03215508,-0.05471073,0.25849187]);
% robot.tool=([-0.00624954,-0.22429878,0.37709918]);
%% 普通机器人的示教展示
 q1 = [312.97,-127.67,-116.68,1.65,39.17,187.07];
 q2 = [318.69,-129.73,-130.53,23.72,31.06,187.06];
 q3 = [303.96,-124.05,-111.19,-7.57,53.16,187.06];
 q4 = [275.14,-120.23,-114.45,2.04,102.37,187.07];
 q5 = [261.87,-129.41,-112.99,10.63,131.26,187.07];
 q6 = [308.47,-127.20,-123.43,15.71,38.45,187.06];

T1 = robot.fkine(deg2rad(q1));
p0 = T1.transl;


T2 = robot.fkine(deg2rad(q2));
p1 = T2.transl;

T3 = robot.fkine(deg2rad(q3));
p2 = T3.transl;
 
T4 = robot.fkine(deg2rad(q4));
p3 = T4.transl ;


T5 = robot.fkine(deg2rad(q5));
p4 = T5.transl;

T6 = robot.fkine(deg2rad(q6)); 
p5 = T6.transl;
%%
v1 = [ 0.0425404 ; 0.05606268; -0.00319922]
v2 = [ 0.08206797; -0.00996652; 0.0078077 ]
v3 = [0.03385289; 0.00124955; 0.00258623]
v4 = [ 0.05182317; 0.03681554; -0.00017162]
v5 =  [ 0.06607809; -0.03130339; 0.00904149] 

nv = cross(v1,v2)
eq1 = abs(dot(nv,v3))
eq2 = abs(dot(nv,v4))
eq3 = abs(dot(nv,v5))
eq1+eq2+eq3


% robot.display();%展示出机器人的信息
% teach(robot);%调出示教滑块

%% 逆解

% % 
% mask=[1,1,1,0,0,0];
% rad2deg( robot.ikine(T,mask))
