#!/usr/bin/env python

'''
Kalman Filters : Example 1
We seek to estimate the position and velocity of an object, of known mass, moving in the gravitational field
We only have access to its position, observed through a noisy sensor, at regular time steps, with a frequency of 10 Hz
'''

import numpy as np
import matplotlib.pyplot as plt

# ------- Physical properties

# The mass of the object
m = 0.1 # kg
g = 9.81 # m/s^2
# True initial position
x0 = 0 
y0 = 0   
 # True Initial velocity 
theta0 = np.pi/4.0 # Angle of the trajectory, in radians
speed0 = 4.0 # Amplitude of the speed
dx0 = speed0 * np.cos(theta0) # m/s
dy0 = speed0 * np.sin(theta0) # m/s

# Time step for the physical simulation
dt = 0.01 # s

# Period of the observations
dt_observations = 0.1 # s

# Definition of the state space equations
# and initialization of the kalman filter
size_observations = 2
size_state = 4
size_control = size_state

xk = np.matrix(np.zeros((size_state,1)))
uk = np.matrix(np.zeros((size_control,1)))
yk = np.matrix(np.zeros((size_observations,)))

Ak = np.matrix(np.array([[1,0,dt_observations,0],[0,1,0,dt_observations],[0,0,1,0],[0,0,0,1]]))
Bk = np.matrix(np.identity(size_state))
Hk = np.matrix(np.array([[1,0,0,0],[0,1,0,0]]))

Pk = np.matrix(np.zeros((size_state, size_state)))
Kk = np.matrix(np.zeros((size_state, size_observations)))

Qk = 1e-4 * np.matrix(np.identity(size_state))
Rk = 1.0 * np.matrix(np.identity(size_observations))

# Initialization of the Kalman filter
xk[0,0] = x0
xk[1,0] = y0
xk[2,0] = dx0
xk[3,0] = dy0



class Simulator:
    '''Simulator of the system'''

    def __init__(self, time_step, mass, gravity):
        self.state = np.matrix(np.array([x0,y0,dx0,dy0])).T
        self.time_step = time_step
        self.A =  np.matrix(np.array([[1,0,time_step,0],[0,1,0,time_step],[0,0,1,0],[0,0,0,1]]))
        self.B = np.matrix(np.array([0,0,0,-mass * gravity * time_step])).T
        self.time = 0

    def single_step(self):
        self.state = self.A * self.state + self.B
        self.time += self.time_step

    def step(self, t):
        #print "Step until ", t
        # Performs some iteration of the system until time t
        while(self.time < t):
            self.single_step()
    
    def get_time(self):
        return self.time

    def get_state(self):
        return self.state


def get_control(uk):
    for i in range(size_control):
        uk[i,0] = 0
    uk[size_control-1,0] = -m*g*dt_observations
    
def get_observations(simulator):
    global Hk
    return Hk* simulator.get_state() +1.0 * 2.0*(np.random.random((size_observations,1))-0.5)

#[xkm, Pkm] = prediction_step(xk, uk, Pk)
def prediction_step(xk, uk, Pk):
    global Ak, Bk, Qk
    return [Ak * xk + Bk* uk,
           Ak * Pk * Ak.T + Qk]

#[xkp, Pkp] = correction_step(xkm, Pkm, yk)
def correction_step(xkm, Pkm, yk):
    global Hk, Rk, Kk
    Kk = Pkm * Hk.T * (Hk * Pkm * Hk.T+Rk).I
    Pkp = (np.matrix(np.identity(size_state)) - Kk * Hk) * Pkm
    xkp = xkm + Kk * (yk - Hk * xkm)
    return [xkp, Pkp]

def simulation(time_limit):
    global xk, yk, Pk
    print "Running the simulation"
    simulator = Simulator(dt, m, g)
    nb_steps = 2+int(time_limit/float(dt_observations))
    res = np.zeros(( nb_steps, 1+size_state+size_observations+size_state))
    epoch = 0
    # Some initialization
    Pk = 1e-1 * np.identity(size_state)
    xk = np.matrix(np.array([0.0, 0.0, 0, 0])).T
    
    for j in range(size_state):
        res[0,j+1] = np.asarray(xk)[j]
    for j in range(size_observations):
        res[0, 1+size_state+j] = 0
    for j in range(size_state):
        res[0,1+size_state+size_observations+j] = np.asarray(simulator.get_state())[j]

    # Perform a prediction step
    for i in range(1,nb_steps):
        t = dt_observations*float(i)
        simulator.step(t)
        yk = get_observations(simulator)
        [xk, Pk] = kalman_step(xk, Pk, yk)
        res[i,0] = t
        for j in range(size_state):
            res[i,j+1] = np.asarray(xk)[j]
        for j in range(size_observations):
            res[i, 1+size_state+j] = yk[j]
        for j in range(size_state):
            res[i,1+size_state+size_observations+j] = np.asarray(simulator.get_state())[j]
    return res

def kalman_step(xk, Pk, yk):
    global uk
    get_control(uk)
    #print uk
    [xkm, Pkm] = prediction_step(xk, uk, Pk)
    # Perform a correction step
    [xkp, Pkp] = correction_step(xkm, Pkm, yk)
    return [xkp, Pkp]

def gen_trajectory(time_limit):
    Ndata = 1+int(time_limit/dt)
    data = np.zeros((Ndata,3))
    s = Simulator(dt, m, g)
    for i in range(Ndata):
        s.step(float(i)*time_limit/(Ndata-1))
        st = s.get_state()
        data[i,0] = float(i)*time_limit/(Ndata-1)
        data[i,1] = np.asarray(st)[0]
        data[i,2] = np.asarray(st)[1]
    print "Final time : ", data[Ndata-1,0]
    return data


# --- main
if __name__ == '__main__':
    res_kalman = simulation(5)
    res_trajectory = gen_trajectory(5)
    
    # Printing the result
    fig = plt.figure(figsize=(10,10))
    ax1 = fig.add_subplot(111)

    # Plotting the estimated state
    ax1.plot(res_kalman[:,1], res_kalman[:,2],linewidth=2)
    # Plotting the observations
    ax1.plot(res_kalman[:,5], res_kalman[:,6],'o')    
    # Plotting the real state
    ax1.plot(res_trajectory[:,1], res_trajectory[:,2],linewidth=2)

    plt.xlabel("x position",fontsize=28)
    plt.ylabel("y position",fontsize=28)
    plt.title("Ball position",fontsize=28)
    
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend(['Estimation','Observation', 'Real'])
    #plt.savefig('Illustration.png',dpi=300)

    plt.figure()
    plt.plot(res_kalman[:,1],res_kalman[:,7])


    plt.show()




