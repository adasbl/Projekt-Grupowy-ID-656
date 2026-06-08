import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

"""
Obliczenia zmiany położenie robota: odometria

Mamy do dyspozycji 4 czujniki pomiarowe:
-dwa enkodery dla każdego z kół
-akcelerometr do pomiaru przyspieszenie robota
-żyroskop do pomiaru prędkości kątowej robota

Zaczniemy od przetworzenia danych z enkoderów:
(wyprowadzenie wzorów i uzupełnienie znadują się na stronie 76 'Elementy Robotyki')

Mając pomiar prędkości obrotowej z kół w_i możemy policzyć przemieszczenie koła ze wzoru:

    d_i = 2*pi*r*w_i(t)*dt ; i:l,r

    r - promień koła[cm],
    w_i(t) - pedkosc oborotwa koła [obr/s],
    dt - próbkowkowanie czasu, co ile wierzymy prędkość [s].

Robot posiada trzy stopnie swobody polozenie wzdłóż osi x, y oraz kąt oborotu wokół własnej osi
Inczej możemy nazwać to jego posturą zapisując: (x[cm],y[cm],phi[stopnie])

Żeby policzyć zmianę postury musimy przyjąć następujace założenie:

    -przemieszczenie jest dość małe żeby móc przybliżyć wartość kąta wartościami długości
    -wynika z powyzszego ze musimy mieć częsty pomiar prędkości obrotowej

Wzory: 

    1) theta = (d_r - d_l)/b 

     theat - kąt o jaki zmienia swoją posture robot,
     b - odległość miedzy kołami

    2) d_c = (d_l +d_r)/2

     d_c - wartość przemieszczenie środka robota

    3) d_x = d_c*cos(phi+theta)

     d_x - zmiana polozenie wzloz osi x

    4) d_y = d_c*sin(phi+theta)

"""

class odometryAlgorithm:
    def __init__ (self,
                  dataFromAccelerator: np.array, #rad/s
                  dataFromGyroscope: np.array,   #rad/s
                  dataFromEncoders: np.array,    #m/s^2
                  startPosture,
                  timeStep):
        
        self.posX = np.array([startPosture[0]]) #cm
        self.posY = np.array([startPosture[1]]) #cm
        self.phi = np.array([startPosture[2]])  #stopnie

        self.dataFromAccelerator = dataFromAccelerator
        self.dataFromGyroscope   = dataFromGyroscope
        self.dataFromEncoders    = {
                        'rightWheel' : dataFromEncoders[0],
                         'leftWheel' : dataFromEncoders[1]
                        }

        self.timeStep = timeStep
        self.distanceLeftWheel = 0
        self.distanceRightWheel = 0
        self.vecTemporaryDistanceLW = np.array([])
        self.vecTemporaryDistanceRW = np.array([])
        self.radiusOfWheel = 5
        self.distanceBetweenwheels = 20

    def calculatePathEncoders(self):
        for i in range(min(len(self.dataFromEncoders['rightWheel']),len(self.dataFromEncoders['leftWheel']))):
            dl = np.pi * self.radiusOfWheel * 2 * self.dataFromEncoders['leftWheel'][i] * self.timeStep
            dr = np.pi * self.radiusOfWheel * 2 * self.dataFromEncoders['rightWheel'][i] * self.timeStep
            self.vecTemporaryDistanceLW = np.append(self.vecTemporaryDistanceLW,dl)
            self.vecTemporaryDistanceRW = np.append(self.vecTemporaryDistanceRW,dr)
        
        for i in range(len(dataFromLeftWheel)):
            #theta = (d_r - d_l)/b
            theta = (self.vecTemporaryDistanceRW[i]-self.vecTemporaryDistanceLW[i])/self.distanceBetweenwheels
            self.phi = np.append(self.phi,self.phi[i]+theta)

            #d_c = (d_l +d_r)/2
            d_c = (self.vecTemporaryDistanceRW[i]+self.vecTemporaryDistanceLW[i])/2

            d_x = d_c * np.cos((self.phi[i]+theta)*(np.pi/180))
            d_y = d_c * np.sin((self.phi[i]+theta)*(np.pi/180))
            self.posX = np.append(self.posX,self.posX[i]+d_x)
            self.posY = np.append(self.posY,self.posY[i]+d_y)

            #print(f"postura: (x:{self.posX[i]}, y:{posY[i]}, phi:{phi[i]})",end='\n')

    def calculatePathINS(self):

        acc = self.dataFromAccelerator
        gyr = self.dataFromGyroscope
        velosityVect = [0]
        thetaVect = [0]

        for i in range(min(len(self.dataFromAccelerator),len(self.dataFromGyroscope))):
            dV = acc[i]*self.timeStep
            velosityVect.append(velosityVect[-1]+dV)
            dTheta = gyr[i]*self.timeStep
            thetaVect.append(thetaVect[-1]+dTheta)

        shiftVect = np.array([velosityVect]) * self.timeStep
        angleVect = [self.phi]

        for i in range(len(thetaVect)):
            angleVect.append(angleVect[-1]+thetaVect[i])

        for shift,theta in zip(shiftVect,angleVect):
            dx = np.cos(theta) * shift
            dy = np.sin(theta) * shift
            self.posX = np.append(self.posX,self.posX[-1]+dx)
            self.posY = np.append(self.posY,self.posY[-1]+dy)
            self.phi = np.degrees(angleVect)

    def visPath(self):

        fig,ax = plt.subplots(figsize=(8,8))

        margin = 20
        ax.set_xlim(min(self.posX)-margin, max(self.posX)+margin)
        ax.set_ylim(min(self.posY)-margin, max(self.posY)+margin)

        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title("Wizulaziacja ruchu robota")
        ax.set_xlabel("X [cm]")
        ax.set_ylabel("Y [cm]")

        robotPath, = ax.plot([],[],'b-',linewidth = 1, label='Ścierzka')
        pathPoints, = ax.plot([],[],'ro',markersize = 8, label='Ścierzka')
        robotNose, = ax.plot([],[],'r-',linewidth = 2)

        plt.legend()

        noseLenght = 5 #cm

        def update(frame):

            robotPath.set_data(self.posX[:frame], self.posY[:frame])
            pathPoints.set_data([self.posX[frame]],[self.posY[frame]])

            noseCorner = np.radians(self.phi[frame])

            noseX = self.posX[frame] + noseLenght * np.cos(noseCorner)
            noseY = self.posY[frame] + noseLenght * np.sin(noseCorner)

            robotNose.set_data([self.posX[frame],noseX],[self.posY[frame],noseY])

            return robotPath, pathPoints, robotNose

        animation = FuncAnimation(fig,update,frames = len(self.posX),interval = 20, blit=True)

        plt.show()
    

# Konfiguracja czasu (tak jak miałeś ostatnio)
timeStep = 0.01
left_1 = np.ones(100) * 1.0
right_1 = np.ones(100) * 1.0
left_2 = np.ones(200) * -15
right_2 = np.ones(200) * 15
left_3 = np.ones(200) * 1.5  # Szybko
right_3 = np.ones(200) * 0.8 # Wolniej
dataFromLeftWheel = np.concatenate([left_1, left_2, left_3])
dataFromRightWheel = np.concatenate([right_1, right_2, right_3])
dataEncoders = np.array([dataFromRightWheel,dataFromLeftWheel])

# Konfiguracja
steps = 100
timeStep = 0.01

# 1. Jazda do przodu (1s: stałe przyspieszenie, gyro 0)
acc_part1 = np.ones(steps) * 5 
gyro_part1 = np.zeros(steps)

# 2. Zakręt (1s: mniejsze przyspieszenie, stałe gyro)
acc_part2 = np.ones(steps) * 2
gyro_part2 = np.ones(steps) * 0.5

# Łączone dane
dataFromAccelerator = np.concatenate([acc_part1, acc_part2])
dataFromGyroscope = np.concatenate([gyro_part1, gyro_part2])

simulation = odometryAlgorithm(
    dataFromAccelerator = dataFromAccelerator,
    dataFromGyroscope = dataFromGyroscope,
    dataFromEncoders = dataEncoders,
    startPosture = [100.0,0.0,90.0],
    timeStep = 0.01
)

simulation.calculatePathINS()
simulation.visPath()