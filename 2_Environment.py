import matplotlib.pyplot as plt
import numpy as np
import random
import pandas as pd

class Highway:
    def __init__(self, figure1=0, figure2=0):  # 初始化
        self.carsnumber = 1 + 6  # number of surrounding Cars
        self.Vel = np.empty([self.carsnumber, 1])  # 车速km/h，顺序为自车、012车道前车、012车道后车
        self.TarVel = np.empty([self.carsnumber, 1])  # 目标车速km/h，顺序上同
        self.PosX = np.empty([self.carsnumber, 1])  # 全局纵向位置m，顺序上同
        self.PosY = np.empty([self.carsnumber, 1])  # 全局横向位置m，顺序上同
        self.EgoLaneID = 0 # 自车所处车道ID，三车道下最右车道为0，最左车道为2
        self.f1 = figure1  # 是否画场景俯视图标志位
        self.f2 = figure2  # 是否画观测数据列表图标志位
        self.LaneWidth = 3.75
        # 画换道路线


    def reset(self):  # 重置场景
        t = 1

    def run(self, action):  # 每步运行
        s = self.observer()

        next_s = self.observer()
        d = self.done()
        r = self.reward(d, action, accel)
        return next_s, r, d

    def observer(self):  # 观测量
        ob = np.empty([1,13]) # 自车视角右前、正前、左前、右后、正后、左后车相对位置，上同顺序车相对速度，自车车速
        ob[12] = self.Vel[0]
        orm = self.sort(self.EgoLaneID)
        if orm==1:
            ob[1] = 200
            ob[4] = self.PosX[self.EgoLaneID+1]-self.PosX[0]
            ob[7] = 10
            ob[10] = self.Vel[self.EgoLaneID+1]-self.Vel[0]
        elif orm==2:
            ob[1] = self.PosX[self.EgoLaneID+1]-self.PosX[0]
            ob[4] = self.PosX[self.EgoLaneID+4]-self.PosX[0]
            ob[7] = self.Vel[self.EgoLaneID+1]-self.Vel[0]
            ob[10] = self.Vel[self.EgoLaneID+4]-self.Vel[0]
        else:
            ob[1] = self.PosX[self.EgoLaneID+4]-self.PosX[0]
            ob[4] = -200
            ob[7] = self.Vel[self.EgoLaneID+4]-self.Vel[0]
            ob[10] = -10
        if self.EgoLaneID == 0:
            ob[0] = 0
            ob[3] = 0
            ob[6] = 0
            ob[9] = 0
        else:
            orr = self.sort(self.EgoLaneID-1)
            if orr == 1:
                ob[0] = 200
                ob[3] = self.PosX[self.EgoLaneID] - self.PosX[0]
                ob[6] = 10
                ob[9] = self.Vel[self.EgoLaneID] - self.Vel[0]
            elif orr == 2:
                ob[0] = self.PosX[self.EgoLaneID] - self.PosX[0]
                ob[3] = self.PosX[self.EgoLaneID + 3] - self.PosX[0]
                ob[6] = self.Vel[self.EgoLaneID] - self.Vel[0]
                ob[9] = self.Vel[self.EgoLaneID + 3] - self.Vel[0]
            else:
                ob[0] = self.PosX[self.EgoLaneID + 3] - self.PosX[0]
                ob[3] = -200
                ob[6] = self.Vel[self.EgoLaneID + 3] - self.Vel[0]
                ob[9] = -10
        if self.EgoLaneID == 2:
            ob[2] = 0
            ob[5] = 0
            ob[8] = 0
            ob[11] = 0
        else:
            orl = self.sort(self.EgoLaneID+1)
            if orl == 1:
                ob[2] = 200
                ob[5] = self.PosX[self.EgoLaneID + 2] - self.PosX[0]
                ob[8] = 10
                ob[11] = self.Vel[self.EgoLaneID + 2] - self.Vel[0]
            elif orl == 2:
                ob[2] = self.PosX[self.EgoLaneID + 2] - self.PosX[0]
                ob[5] = self.PosX[self.EgoLaneID + 5] - self.PosX[0]
                ob[8] = self.Vel[self.EgoLaneID + 2] - self.Vel[0]
                ob[11] = self.Vel[self.EgoLaneID + 5] - self.Vel[0]
            else:
                ob[2] = self.PosX[self.EgoLaneID + 5] - self.PosX[0]
                ob[5] = -200
                ob[8] = self.Vel[self.EgoLaneID + 5] - self.Vel[0]
                ob[11] = -10
        return ob

    def sort(self, lane): # 车辆位置排序
        return 123 #1代表自车位于最前方，2代表自车位于中间，3代表自车位于最后方

    def reward(self, done, action, accel):  # 计算奖励函数
        if done==1:
            return -20
        r = accel
        if action!=0:
            r=r-1
        if self.Vel[0]<60:
            r = r-0.1
        return r


    def done(self):  # 中止标志位
        t = 1

    def plot1(self):  # 画场景俯视图
        t = 1

    def plot2(self):  # 画观测数据列表图
        t = 1
        observer
        reward
        done


    def acc(self, id1, id2):
        id1.v,tv
        id2.v
        reldis
        return id1.a

    def longi(self):
        id 0-6
