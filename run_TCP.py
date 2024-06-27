import numpy as np
from calibration_utils import tcp_cali


def run():
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
                positions_str = line.split('Joint Positions:')[1].strip().strip('[]')
                # 将字符串转换为 float 列表
                positions = tuple(map(float, positions_str.split(', ')))
                # 添加到总列表中
                all_joint_positions.append(list(positions))


    print(all_joint_positions)
    data = tcp_cali(all_joint_positions)
    data.run()


if __name__ == '__main__':
    run()





'''
find solution:  [[-0.0093143 ]
 [-0.20472612]
 [ 0.43112701]]
current best average dist:  2.5439500821596988e-17
current max dist:  5.550436542893888e-17
current min dist:  1.387609135723472e-17

'''


'''

"针头"
find solution:  [[ 0.03896421]
 [-0.22570285]
 [ 0.37671283]]
current best average dist:  4.3945024677491445e-17
current max dist:  1.24896385925502e-16
current min dist:  0.0


'''


'''
桌子1,去掉第二个()

find solution:  [[ 0.02974172]
 [-0.21578594]
 [ 0.38169355]]
current best average dist:  4.423653034485612e-06
current max dist:  1.1655874545117222e-05
current min dist:  1.4900950838785246e-06
'''

'''
桌子2,加固tool(TCP位置发生变化)，去掉第四个
find solution:  [[ 0.02573503]
 [-0.20825715]
 [ 0.38056999]]
current best average dist:  6.858201298472371e-06
current max dist:  1.9350731749867313e-05
current min dist:  1.1887639240644134e-06
'''

'''
桌子3,去掉第二个
find solution:  [[ 0.02741757]
 [-0.20873842]
 [ 0.37970762]]
current best average dist:  4.191890422520002e-05
current max dist:  0.00011072085120565273
current min dist:  7.702141223152015e-07

'''

'''
桌子4
current solution : [[ 0.02853575]
 [-0.20811175]
 [ 0.38090266]] current cost 2.2724877535296173e-16
current best average dist:  0.00016130348145457603
current max dist:  0.0003740927926870646
current min dist:  2.8338092652125222e-05

'''



