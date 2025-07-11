import time
import matplotlib.pyplot as plt
from collections import deque
import threading

class PID_Controller:
    def __init__(self, kp, ki, kd, dt=None, draw = False, name="pid controller", max_history=100):
        self.kp = kp  # 比例增益
        self.ki = ki  # 积分增益
        self.kd = kd  # 微分增益
        self.last_error = 0.0
        self.integral = 0.0
        self.dt = dt
        self.last_t = time.time()
        self.name = name
        self.draw = draw
        self.count = 0
        if self.draw:
            plt.ion()  # 开启交互模式
            self.fig, self.ax = plt.subplots(2, 1, figsize=(10, 6), num=self.name)
            self.fig.suptitle(f'PID Controller: {self.name}')
            
            # 初始化数据存储
            self.k_history = deque(maxlen=max_history)
            self.error_history = deque(maxlen=max_history)
            self.output_history = deque(maxlen=max_history)
            
            # 初始化绘图线条
            self.error_line, = self.ax[0].plot([], [], 'r-', label='Error')
            self.output_line, = self.ax[1].plot([], [], 'b-', label='Output')
            
            # 设置子图
            self.ax[0].set_ylabel('Error')
            self.ax[0].grid(True)
            self.ax[0].legend()
            
            self.ax[1].set_xlabel('k')
            self.ax[1].set_ylabel('Output')
            self.ax[1].grid(True)
            self.ax[1].legend()
    
    def init_time(self):
        self.last_t = time.time()

    def __call__(self, error):
        if self.dt is None:
            t = time.time()
            dt = t - self.last_t
            self.last_t = t
        else:
            dt = self.dt
        # print(f"dt: {dt}")

        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral
        d = self.kd * (error - self.last_error) / dt
        self.last_error = error
        out = p + i + d

        if self.draw:
            self.k_history.append(self.count)
            self.error_history.append(error)
            self.output_history.append(out)
            
            # 更新绘图数据
            self.error_line.set_data(self.k_history, self.error_history)
            self.output_line.set_data(self.k_history, self.output_history)
            
            # 调整坐标轴范围
            for ax in self.ax:
                ax.relim()
                ax.autoscale_view()
            
            # 重绘图形
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        self.count += 1

        return out
    

if __name__ == "__main__":
    controller = PID_Controller(0.2,0.1,0, draw=True)
    # controller = PID_Controller(0.1,0.1,0, dt=0.01)

    erro = 1
    controller.init_time()
    s = 0
    while True:
        action = controller(erro)
        s = 2*action
        erro = -1-s
        print(erro)
        time.sleep(0.1)
