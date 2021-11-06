import matplotlib.pyplot as plt
import numpy as np
import random
import pandas as pd

class Highway:
    def __init__(self, figure1=0, figure2=0):  # 初始化
        self.Vel = np.empty([1, 7])  # 车速km/h，顺序为自车、左前车、正前车、右前车、左后车、正后车、右后车
        self.TarVel = np.empty([1, 7])  # 目标车速km/h，顺序为自车、左前车、正前车、右前车、左后车、正后车、右后车
        self.PosX = np.empty([1, 7])  # 全局纵向位置m，顺序上同
        self.PosY = np.empty([1, 7])  # 全局横向位置m，顺序上同
        self.TarVel = np.empty([1, 6])
        self.EgoLaneID = 0 # 自车所处车道ID
        f1 = figure1  # 是否画场景俯视图标志位
        f2 = figure2  # 是否画观测数据列表图标志位
        # 画换道路线

    def reset(self):  # 重置场景
        t = 1

    def run(self, action):  # 每步运行
        s = self.observer()

        next_s = self.observer()
        d = self.done()
        r = self.reward()
        return next_s, r, d

    def observer(self):  # 观测量
        t = 1

    def reward(self):  # 计算奖励函数
        t = 1

    def done(self):  # 中止标志位
        t = 1

    def plot1(self):  # 画场景俯视图
        t = 1

    def plot2(self):  # 画观测数据列表图
        t = 1