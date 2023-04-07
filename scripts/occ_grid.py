import numpy as np
import matplotlib.pyplot as plt
import random
import math
import time
import matplotlib.image as mpimg
import networkx as nx
from skimage.draw import line
import keyboard
import sys

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')


class LaserData:
    dist = 0
    angle = 0
    x = 0
    y = 0
    XOffset = 0
    YOffset = 0

    def __init__(self, distance, angle_beam, x_obs, y_obs):
        self.dist = distance
        self.angle = angle_beam
        self.x = x_obs
        self.y = y_obs


class OccupancyGridMapping:
    cell_size = 1
    map_size = np.array([40, 40])
    m = []
    rows = 0
    cols = 0

    def __init__(self, cell_size, map_size):
        self.cell_size = cell_size
        self.map_size = map_size
        self.rows, self.cols = (self.map_size / self.cell_size).astype(int)
        self.m = np.random.uniform(low=0.5, high=0.5, size=(self.rows, self.cols))

    def log_odds(self, p):
        return math.log10(p/(1-p))

    def inverse_sensor_model(self, i, j, r, robot_x, robot_y):
        #distance from sensor to cell
        cell_x, cell_y = np.floor((1/self.cell_size)*np.array([i,j]))
        d = np.sqrt((robot_x-cell_x)**2 + (robot_y-cell_y)**2)
        if(d > (r+0.5)):
            return 0

        elif(d <= (r+0.5) and d >= (r-0.5)):
            #this cell probably contains an obstacle
            #return self.log_odds(0.75)
            return 0.2
        elif(d < (r-0.5)):
            #this cell is "before" the obstacle, so it must be free
            #return self.log_odds(0.25)
            return -0.2

    def update_cell(self, j, i, r, robot_x, robot_y):
        l_ti = self.log_odds(self.m[i,j]) + self.inverse_sensor_model(i, j, r, robot_x, robot_y) + self.log_odds(0.5)
        p_m = 1 - (1/(1+math.exp(l_ti)))
        self.m[i,j] = p_m

    def format_laser_data(self, clientID, x_robot, y_robot, yaw):
        laser_data = []
        returnCodeRanges, string_range_data = sim.simxGetStringSignal(clientID, "hokuyo_range_data", sim.simx_opmode_streaming)
        returnCodeAngles, string_angle_data = sim.simxGetStringSignal(clientID, "hokuyo_angle_data", sim.simx_opmode_blocking)
        ax = fig.add_subplot(111, aspect='equal')
        if returnCodeRanges == 0 and returnCodeAngles == 0:
            # unpack data from range and sensor messages
            raw_range_data = sim.simxUnpackFloats(string_range_data)
            raw_angle_data = sim.simxUnpackFloats(string_angle_data)
            # calculate x,y coordinates of an obstacle based on beam distance and angle
            for i in range(len(raw_range_data)):
                x = (raw_range_data[i] * np.cos(raw_angle_data[i]+yaw))+x_robot
                y = (-raw_range_data[i] * np.sin(raw_angle_data[i]+yaw))+y_robot
                if(x > self.map_size[0] or y > self.map_size[1]):
                    continue

                data = LaserData(raw_range_data[i], raw_angle_data[i], x, y)
                laser_data.append(data)
            plt.show()
            return laser_data

        # return none in case were nothing was gotten from the simulator
        return None

map_size = np.array([50, 50])
cell_size = 0.2

occ_grid_map = OccupancyGridMapping(cell_size, map_size)
fig = plt.figure(figsize=(8,8), dpi=100)
ax = fig.add_subplot(111, aspect='equal')



plt.imshow(occ_grid_map.m, cmap='Greys', origin='upper', extent=(0, occ_grid_map.cols, occ_grid_map.rows, 0))

ax.set_xticks(np.arange(0, occ_grid_map.cols, cell_size))
ax.set_yticks(np.arange(0, occ_grid_map.rows, cell_size))

#plt.ion()
plt.colorbar()
plt.show()

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print('Connected to remote API server')

    # Handle para o ROBÔ e frame de referência inercial
    robotname = 'Pioneer_p3dx'
    returnCode, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)
    returnCode, refHandle = sim.simxGetObjectHandle(clientID, 'globalFrame', sim.simx_opmode_oneshot_wait)
    # Handle para as juntas das RODAS
    returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

    # Handle para os dados do LASER
    laser_data_name = "hokuyo_range_data"

    # Geralmente a primeira leitura é inválida (atenção ao Operation Mode)
    # Em loop até garantir que as leituras serão válidas
    returnCode = 1
    while returnCode != 0:
        returnCode, range_data = sim.simxGetStringSignal(clientID, laser_data_name, sim.simx_opmode_streaming + 10)

    # Prosseguindo com as leituras
    #laser_data = format_laser_data(clientID)

    returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)

    # raise SystemExit()

    # Dados do Pioneer
    L = 0.381  # Metros
    r = 0.0975  # Metros

    t = 0
    # Lembrar de habilitar o 'Real-time mode'
    startTime = time.time()
    lastTime = startTime

    while not keyboard.is_pressed('q'):
        now = time.time()
        dt = now - lastTime
        returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, refHandle, sim.simx_opmode_oneshot_wait)
        returnCode, ori = sim.simxGetObjectOrientation(clientID, robotHandle, refHandle, sim.simx_opmode_oneshot_wait)
        # Fazendo leitura do laser
        laser_data = occ_grid_map.format_laser_data(clientID, pos[0], pos[1], ori[2])
        #fig2 = plt.figure(figsize=(8, 8), dpi=100)
        #ax2 = fig2.add_subplot(111, aspect='equal')
        #ax2.set_xticks(np.arange(0, occ_grid_map.cols, cell_size))
        #ax2.set_yticks(np.arange(0, occ_grid_map.rows, cell_size))

        for laser in laser_data:
            xi, yi = np.floor((1 / cell_size) * np.array([pos[0], pos[1]])).astype(int)
            xoi, yoi = np.floor((1 / cell_size) * np.array([laser.x, laser.y])).astype(int)
            #print(xoi, yoi)

            line_bresenham = np.zeros((occ_grid_map.rows, occ_grid_map.cols), dtype=np.uint8)
            rr, cc = line(yi, xi, yoi, xoi)  # r0, c0, r1, c1
            #print(rr,cc)
            #print(rr,cc)
            line_bresenham[rr, cc] = 1

            aux = np.zeros(shape=(occ_grid_map.rows,occ_grid_map.cols))
            aux[yi,xi] = 1
            for i in range(0, len(rr)):
                occ_grid_map.update_cell(rr[i], cc[i], laser.dist, xi, yi)


            #ax2.imshow(line_bresenham, cmap='Reds', extent=(0, occ_grid_map.cols, occ_grid_map.rows, 0), alpha=.5)
            #ax2.imshow(aux, cmap='Blues', extent=(0,occ_grid_map.cols,occ_grid_map.rows,0), alpha=1.0)

            #ax.set_xticks(np.arange(0, occ_grid_map.cols, cell_size))
            #ax.set_yticks(np.arange(0, occ_grid_map.rows, cell_size))
            #plt.show()
            #ax.plot([pos[0], laser.x], [pos[1], laser.y], 'r-', linewidth=3)
            #fig.canvas.draw()

        if keyboard.is_pressed('w'):
            v = 0.7
        elif keyboard.is_pressed('s'):
            v = -0.7
        else:
            v = 0.0

        if keyboard.is_pressed('a'):
            w = 0.5
        elif keyboard.is_pressed('d'):
            w = -0.5
        else:
            w = 0.0

        wl = v / r - (w * L) / (2 * r)
        wr = v / r + (w * L) / (2 * r)

        # Enviando velocidades
        sim.simxSetJointTargetVelocity(clientID, l_wheel, wl, sim.simx_opmode_streaming + 5)
        sim.simxSetJointTargetVelocity(clientID, r_wheel, wr, sim.simx_opmode_streaming + 5)

        t = t + dt
        lastTime = now

    # Parando o robô
    sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

    # Parando a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

    fig = plt.figure(figsize=(8, 8), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')
    plt.imshow(occ_grid_map.m, cmap='Greys', origin='upper', extent=(0, occ_grid_map.cols, occ_grid_map.rows, 0))
    ax.set_xticks(np.arange(0, occ_grid_map.cols, cell_size))
    ax.set_yticks(np.arange(0, occ_grid_map.rows, cell_size))
    plt.colorbar()

    plt.show()

else:
    print('Failed connecting to remote API server')

print('Program ended')
np.set_printoptions(threshold=sys.maxsize)

#print(occ_grid_map.m)