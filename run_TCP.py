import numpy as np
from calibration_utils import OMOPSO


# 文件名
filename = 'joint_states.txt'

# 初始化一个列表来存储所有的 joint positions
all_joint_positions = []
# 读取文件内容
with open(filename, 'r') as file:
    lines = file.readlines()
    for line in lines:
        # 查找包含 Joint Positions 的行
        if 'Joint Positions:' in line:
            # 提取 joint positions
            positions_str = line.split('Joint Positions:')[1].strip().strip('()')
            # 将字符串转换为 float 列表
            positions = tuple(map(float, positions_str.split(', ')))
            # 添加到总列表中
            all_joint_positions.append(list(positions))
for i in all_joint_positions:
    #读取的时候是按照关节名字的顺序读取的，所以要调整一下顺序
    i[0],i[2]=i[2],i[0]


data = OMOPSO(all_joint_positions)
data.run()