from builtins import range, len
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
# lane information

np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
class Highway:

    def __init__(self, figure1=0, figure2=0):  # 初始化
        self.carsnumber = 1 + 6    # number of surrounding Cars
        self.Vel = np.empty([self.carsnumber, 1])  # 车速km/h，顺序为自车、左前车、正前车、右前车、左后车、正后车、右后车
        self.TarVel = np.empty([self.carsnumber, 1])  # 目标车速km/h，顺序为自车、左前车、正前车、右前车、左后车、正后车、右后车
        self.PosX = np.empty([self.carsnumber, 1])  # 全局纵向位置m，顺序上同
        self.PosY = np.empty([self.carsnumber, 1])  # 全局横向位置m，顺序上同
        self.TarVel = np.empty([self.carsnumber, 1])
        self.EgoTarVel = np.empty([1, 1])
        self.EgoLaneID = 1  # 自车所处车道ID
        self.LaneWidth = 3.5
        self.WhichLane = [0, 1, 2]
        self.LaneRange = np.arange(-200., 200., 0.01)
        self.f1 = figure1  # 是否画场景俯视图标志位
        self.f2 = figure2  # 是否画观测数据列表图标志位
        # 画换道路线
        self.fig = plt.figure(num=1, figsize=(8, 50 / (self.LaneRange[-1] * 2) * len(self.WhichLane) * self.LaneWidth))
        self.ax = self.fig.add_subplot(111)

    secureDistance = 10.

    def plot1(self):
        if self.f1 == 1:
            for i in np.hstack([-1, self.WhichLane]):
                plt.plot(self.LaneRange, np.ones(len(self.LaneRange)) * self.LaneWidth * i, 'r--')
            for i in range(1, self.carsnumber):
                self.ax.add_patch(patches.Rectangle(([self.PosX[i]-self.PosX[0], self.PosY[i] - self.LaneWidth/4]), 5, 2, 0, color="blue"))
            self.ax.add_patch(patches.Rectangle(([0, self.PosY[0] - self.LaneWidth/4]), 5, 2, 0, color="red"))
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("output")
            plt.xlim([self.LaneRange[0], self.LaneRange[-1]])
            self.fig.show()
            plt.pause(0.0001)
            plt.cla()

    def reset(self):
        # initialize lane data
        # collision distance centric Car
        self.Vel[0] = 80 / 3.6  # initialize Car's velocity(unit:metre per second)
        self.EgoTarVel = 80 / 3.6  # initialize Car's expected velocity
        judge = True  # a viriation to judge collision
        # initialize car's data
        self.PosX[0] = 0.
        self.PosY[0] = self.WhichLane[1] * self.LaneWidth - 0.5 * self.LaneWidth  # [lateral location, whichLane]
        while judge == True:
            for i in range(1, self.carsnumber):

                if i < self.carsnumber/2:
                    self.PosY[i] = self.WhichLane[i-1] * self.LaneWidth - 0.5 * self.LaneWidth
                    self.PosX[i] = random.uniform(self.PosX[0], self.LaneRange[-1])
                    self.Vel[i] = random.uniform(50, 90) / 3.6  # initialize surrounding Car's velocity
                    self.TarVel[i] = random.uniform(50, 90) / 3.6  # initialize surrounding Car's expected velocity
                else:
                    self.PosY[i] = self.WhichLane[i-1-self.carsnumber//2] * self.LaneWidth - 0.5 * self.LaneWidth
                    self.PosX[i] = random.uniform(self.LaneRange[0], self.PosX[0])
                    self.Vel[i] = random.uniform(50, 90) / 3.6  # initialize surrounding Car's velocity
                    self.TarVel[i] = random.uniform(50, 90) / 3.6  # initialize surrounding Car's expected velocity
            Location = np.hstack([self.PosX, self.PosY])
            judge = self.done2(Location, self.secureDistance)
        self.plot1()

    def run(self, turn):  # 0 is keep lane, 1 is turn left , 2 is turn right
        dt = 0.1
        # longitudinal control
        a = np.zeros([self.carsnumber, 1])
        place = self.sort(turn)
        # target velocity
        for i in range(0, self.carsnumber):
            if (self.Vel[i] < self.TarVel[i]):
                a[i] = 20
            else:
                a[i] = 0
        # longitudinal control
        for i in self.WhichLane:
            if(self.EgoLaneID == i):
                place = self.sort(i)
                if(place == 1):
                    if (self.done(np.hstack([self.PosX[:2+i:1+i], self.PosY[:2+i:1+i]]), self.secureDistance * 1.5)):
                        a[1+i] = -30
                elif(place == 2):
                    if (self.done(np.hstack([self.PosX[:2+i:1+i], self.PosY[:2+i:1+i]]), self.secureDistance * 1.5)):
                        a[0] = -30
                    if (self.done(np.hstack([self.PosX[:2+i+3:1+i+3], self.PosY[:2+i+3:1+i+3]]), self.secureDistance * 1.5)):
                        a[i+4] = -30
                elif(place == 3):
                    if (self.done(np.hstack([self.PosX[:2+i+3:1+i+3], self.PosY[:2+i+3:1+i+3]]), self.secureDistance * 1.5)):
                        a[0] = -30
            else:
                if (self.done(np.hstack([self.PosX[i+1:i+5:3], self.PosY[i+1:i+5:3]]), self.secureDistance * 1.5)):
                    a[i+4] = -30

        self.PosX[0] = self.Vel[0] * dt + a[0] * 0.5 * np.power(dt, 2) + self.PosX[0]
        self.Vel[0] = self.Vel[0] + a[0] * dt
        self.Vel[1:] = self.Vel[1:] + a[1:] * dt
        self.PosX[1:] = self.Vel[1:] * dt + a[1:] * 0.5 * np.power(dt, 2) + self.PosX[1:]
        dx=self.Vel[0] * dt + a[0] * 0.5 * np.power(dt, 2)
        self.lateral(turn, dx, dt)

        # judge collision and plot
        self.plot1()
        Location = np.hstack([self.PosX, self.PosY])
        judge = self.done2(Location, self.secureDistance)
        reward = 0
        next_s = [self.PosX, self.PosY]
        return next_s, reward, judge

    def lateral(self, turn, xc, Ts):
        yc=self.PosY[0]
        L=180
        pi=3.141592653589793
        if(turn==1): #turn left
            yc = yc + 50*(self.LaneWidth / (2 * pi) * (2 * pi * (xc + Ts * self.Vel[0]) / L + math.sin(2 * pi * (xc + Ts * self.Vel[0]) / L - pi)) - self.LaneWidth / (
                 2 * pi) * (2 * pi * xc / L + math.sin(2 * pi * xc / L - pi)))
        elif(turn==2): #tuen right
            yc = yc - (self.LaneWidth / (2 * pi) * (2 * pi * (xc + Ts * self.Vel[0]) / L + math.sin(2 * pi * (xc + Ts * self.Vel[0]) / L - pi)) - self.LaneWidth / (
                    2 * pi) * (2 * pi * xc / L + math.sin(2 * pi * xc / L - pi)))
        self.PosY[0] = yc
        self.EgoLaneID = int(1.001 + self.PosY[0]//self.LaneWidth)
        if self.EgoLaneID != 1:
            kk=0


    # judge collision
    def done(self, Location, secureDistance):
        judge = False
        for i in range(0 , np.size(Location, 0)):
            for j in range(i + 1, np.size(Location, 0)):
                if np.linalg.norm([Location[i] - Location[j]]) <= secureDistance :
                    judge = True
        return judge

    def done2(self, Location, secureDistance):
        judge = False
        for i in range(0 , np.size(Location, 0)):
            for j in range(i + 1, np.size(Location, 0)):
                if np.linalg.norm([Location[i] - Location[j]]) <= secureDistance and Location[i, 1] == Location[j, 1]:
                    judge = True
        return judge

    def sort(self, lane):
        place = 1
        for i in range(1, self.carsnumber - 1):
            if (((lane - 0.5) * self.LaneWidth == self.PosY[i]) & (self.PosX[0] < self.PosX[i])):
                place += 1
        return place
    '''
    def longitudinalControl (self, place,turn):
        laneId1=[1, 4]
        laneId2=[2, 5]
        laneId3=[3, 6]
        if(self.EgoLaneID == 1):
    '''



# 初始化
ev = Highway(figure1=1,figure2=1)
episodes = 50000
score_list = []

# 每个回合运行
for i in range(episodes):
    step = 1000
    score = 0
    s = ev.reset()

    # 每步运行
    for j in range(step):
        next_s, reward, done = ev.run(1)
        if done:
            score_list.append(score)
            break
