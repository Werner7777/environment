from builtins import range, len
import random

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# judge collision
def done(Location, secureDistance):
    judge =False
    for i in range(0, np.size(Location, 0)):
        for j in range(i + 1, np.size(Location, 0)):
            if np.linalg.norm([Location[i]-Location[j]]) <= 10 and Location[i, 1]==Location[j, 1]:
                judge = True
    return judge

# initialize the carinformation and plot
def initialize(laneInformation):
    # initialize lane data
    surroundingCarsNumber = 6 # number of surrounding Cars
    secureDistance = 10.         # collision distance centric Car
    vCar = 80 / 3.6           # initialize Car's velocity
    vExpCar = 80 / 3.6        # initialize Car's expected velocity
    judge = True     # a viriation to judge collision
    # initialize car's data
    CarLocation = np.array([0.,whichLane[1]* laneWide-0.5*laneWide])  # [lateral location, whichLane]
    surroundingCarsLocation = np.zeros([surroundingCarsNumber, 2])
    vSurroundingCars = np.zeros(surroundingCarsNumber)
    vExpSurroundingCars = np.zeros(surroundingCarsNumber)
    while judge == True:
        for i in range(0, surroundingCarsNumber):
            if i < 3:
                surroundingCarsLocation[i] = [random.uniform(laneRange[0], CarLocation[0]),
                                           random.choice(whichLane)*laneWide-0.5*laneWide] # [longitudinal location, latitudinal location]
                vSurroundingCars[i] = random.uniform(50, 90) / 3.6      # initialize surrounding Car's velocity
                vExpSurroundingCars[i] = random.uniform(50, 90) / 3.6   # initialize surrounding Car's expected velocity
            else:
                surroundingCarsLocation[i] = [random.uniform(CarLocation[0], laneRange[-1]),
                                           random.choice(whichLane)*laneWide-0.5*laneWide]  # [longitudinal location, latitudinal location]
                vSurroundingCars[i] = random.uniform(50, 90) / 3.6      # initialize surrounding Car's velocity
                vExpSurroundingCars[i] = random.uniform(50, 90) / 3.6   # initialize surrounding Car's expected velocity
        judge = done(np.vstack([surroundingCarsLocation, CarLocation]), secureDistance)
    plotInit(CarLocation, surroundingCarsNumber, surroundingCarsLocation, laneInformation, fig)
    carsInformation = [CarLocation, vCar, vExpCar, surroundingCarsNumber,
                       surroundingCarsLocation, vSurroundingCars, vExpSurroundingCars]  # collect information about car and surrounding cars
    return carsInformation
def plotInit(CarLocation, surroundingCarsNumber, surroundingCarsLocation, laneInformation, fig):
    laneRange = np.arange(CarLocation[0]-200., CarLocation[0]+200., 0.01)
    laneWide = laneInformation[1]
    whichLane = laneInformation[2]
    #ax = fig.add_subplot(111)
    for i in np.hstack([-1, whichLane]):
        plt.plot(laneRange, np.zeros(len(laneRange)), 'r--')
        plt.plot(laneRange, np.ones(len(laneRange)) * laneWide * i, 'r--')
    for i in range(0, surroundingCarsNumber):
        ax.add_patch(patches.Rectangle((surroundingCarsLocation[i] - 1), 5, 2, 0, color="blue"))
    ax.add_patch(patches.Rectangle((CarLocation - 1),5 ,2,0,color="red"))
    plt.xlabel("X-AXIS")
    plt.ylabel("Y-AXIS")
    plt.title("PLOT-1")
    plt.xlim([laneRange[0], laneRange[-1]])
    fig.show()
    plt.pause(0.003)
    plt.cla()
    '''
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    line1, = ax.plot(x, y, 'b-')

    for phase in np.linspace(0, 10*np.pi, 100):
        line1.set_ydata(np.sin(0.5 * x + phase))
        fig.canvas.draw()
        fig.canvas.flush_events()
    '''
def run(carsInformation, dt, laneInformation, secureDistance, fig):
    ## extract the information of lane and cars
    ##laneRange =laneInformation[0]
    ##laneWide = laneInformation[1]
    ##whichLane = laneInformation[2]

    CarLocation=carsInformation[0]
    vCar=carsInformation[1]
    vExpCar=carsInformation[2]
    surroundingCarsNumber=carsInformation[3]
    surroundingCarsLocation=carsInformation[4]
    vSurroundingCars=carsInformation[5]
    vExpSurroundingCars=carsInformation[6]

    # longitudinal control
    aSurroundingCars = np.zeros(surroundingCarsNumber)
    if (vCar < vExpCar):
        aCar = 0.01
    else:
        aCar = 0
    for i in range(0, surroundingCarsNumber):
        if(vSurroundingCars[i] < vExpSurroundingCars[i]):
            aSurroundingCars[i] = 0.01
        else:
            aSurroundingCars[i] = 0
    CarLocation[0] = vCar * dt + aCar * 0.5 * np.power(dt,2) + CarLocation[0]
    vCar = vCar + aCar * dt
    vSurroundingCars = vSurroundingCars +  aSurroundingCars * dt
    surroundingCarsLocation[:,0] = vSurroundingCars * dt + aCar * 0.5 * np.power(dt,2) + surroundingCarsLocation[:, 0]

    # judge collision and plot
    judge = done(np.vstack([surroundingCarsLocation, CarLocation]), secureDistance)
    plotInit(CarLocation, surroundingCarsNumber, surroundingCarsLocation, laneInformation, fig)
    # return the new carinformation
    carsInformation = [CarLocation, vCar, vExpCar, surroundingCarsNumber,
                       surroundingCarsLocation, vSurroundingCars,
                       vExpSurroundingCars]  # collect information about car and surrounding cars
    return [judge, carsInformation]

# initialize lane information
laneRange = np.arange(-200., 200., 0.01)  # range of lane
laneWide = 3.75  # wide of lane
whichLane = [0, 1, 2]  # number the lane
laneInformation = [laneRange, laneWide, whichLane]
secureDistance = 10

fig= plt.figure()
ax = fig.add_subplot(111)
carsInformation = initialize(laneInformation)

for dt in np.linspace(0., 10000., 1000000):
    [judge, carsInformation] = run(carsInformation, dt, laneInformation, secureDistance, fig)
    if(judge==True):
        # initialize(laneInformation)
        print("car collision")
        break