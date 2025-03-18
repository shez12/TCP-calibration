import numpy as np
from calibration_utils import tcp_cali




def read_data(filename):
    all_joint_positions = []
    # 读取文件内容
    with open(filename, 'r') as file:
        lines = file.readlines()
        for line in lines:
            # 查找包含 Joint Positions 的行
            if 'Joint Positions:' in line:
                # 提取 joint positions
                positions_str = line.split('Joint Positions:')[1].strip().strip('[]')
                # 将字符串转换为 float 列表
                positions = tuple(map(float, positions_str.split(', ')))
                # 添加到总列表中
                all_joint_positions.append(list(positions))
    return all_joint_positions



def run():
    # 文件名
    filename = 'joint_states.txt'

    # 读取数据
    all_joint_positions = read_data(filename)
    print(all_joint_positions)
    data = tcp_cali(all_joint_positions)
    data.run()


if __name__ == '__main__':
    run()




