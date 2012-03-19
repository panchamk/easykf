#!/usr/bin/env python

'''
Kalman Filters : Example 1
We seek to estimate the position and velocity of an object, of known mass, moving in the gravitational field
We only have access to its position, observed through a noisy sensor, at regular time steps, with a frequency of 10 Hz
'''

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import svd
from matplotlib.patches import Ellipse

class Simulator:
    '''Simulator of the system'''

    def __init__(self, params, initial_state):
        ''' params = [time_step, gravity , mass] '''
        self.time_step = params[0]
        self.gravity = params[1]
        self.mass = params[2]
        self.initial_state = initial_state
        self.A =  np.matrix(np.array([[1,0,self.time_step,0],[0,1,0,self.time_step],[0,0,1,0],[0,0,0,1]]))
        self.u = np.matrix(np.array([0,0,0,-self.mass * self.gravity * self.time_step])).T
        self.state = np.matrix(self.initial_state).T

    def init(self):
        self.state = np.matrix(self.initial_state).T
        self.time = 0

    def single_step(self):
        self.state = self.A * self.state + self.u
        self.time += self.time_step

    def loop_until(self, t):
        # Performs some iteration of the system until time t
        while(self.time < t):
            self.single_step()
    
    def get_state(self):
        return self.state

    def get_control(self):
        return self.u

    def gen_trajectory(self,time_limit):
        self.init()
        Ndata = 1+int(time_limit/self.time_step)
        data = np.zeros((Ndata,3))
        for i in range(Ndata):
            self.loop_until(float(i)*time_limit/(Ndata-1))
            st = self.get_state()
            data[i,0] = self.time
            data[i,1] = np.asarray(st)[0]
            data[i,2] = np.asarray(st)[1]
        return data


class KalmanFilter:
    '''Kalman filter for linear state space'''
    
    def __init__(self, params, size_state, size_control, size_observations, simulator, evolution_noise, observation_noise):
        ''' params = [dt, gravity, mass]'''
        self.time_step = params[0]
        self.gravity = params[1]
        self.mass = params[2]
        self.size_state = size_state
        self.size_control = size_control
        self.size_observations = size_observations
        self.Ak =  np.matrix(np.array([[1,0,self.time_step,0],[0,1,0,self.time_step],[0,0,1,0],[0,0,0,1]]))
        self.Bk = np.matrix(np.zeros((size_state, size_control)))
        for i in range(min(size_state, size_control)):
            self.Bk[i,i] = 1.0
        self.xk = np.matrix(np.zeros((size_state,1)))
        self.uk =  np.matrix(np.array([0,0,0,-self.mass * self.gravity * self.time_step])).T
        self.yk = np.matrix(np.zeros((size_observations,1)))

        self.Hk = Hk = np.matrix(np.array([[1,0,0,0],[0,1,0,0]]))

        self.Pk = np.matrix(np.zeros((size_state, size_state)))
        self.Kk = np.matrix(np.zeros((size_state, size_observations)))

        self.Qk = evolution_noise * np.matrix(np.identity(size_state))
        self.Rk = observation_noise * np.matrix(np.identity(size_observations))

        self.simulator = simulator
    
    def init_state_from_array(self, xk, initial_covariance):
        self.xk = np.matrix(xk).T
        self.Pk = initial_covariance * np.identity(self.size_state)
        self.time = 0

    def get_observations(self):
        self.yk = self.Hk* self.simulator.get_state() + np.random.normal(0.0,0.8,(self.size_observations,1))
        
    def prediction_step(self):
        self.xkm = self.Ak * self.xk + self.Bk * self.uk
        self.Pkm = self.Ak * self.Pk * self.Ak.T + self.Qk

    def correction_step(self):
        self.Kk = self.Pkm * self.Hk.T * (self.Hk * self.Pkm * self.Hk.T+self.Rk).I
        self.Pk = self.Pkm - self.Kk * self.Hk * self.Pkm
        self.xk = self.xkm + self.Kk * (self.yk - self.Hk * self.xkm)

    def get_results(self):
        res = np.array([self.time])
        res = np.append(res, np.asarray(self.xk))
        for j in range(self.size_state):
            res = np.append(res, self.Pk[j,j])
        res = np.append(res,  np.asarray(self.yk))
        res = np.append(res,  np.asarray(self.simulator.get_state()))
        return res

    def simulation(self, time_limit):
        print "Running the simulation"
        nb_steps = 2+int(time_limit/float(self.time_step))
        res = np.zeros(( nb_steps, 1+2*self.size_state+self.size_observations+self.size_state))
        # Save the initial time, state estimate, covariance, simulator state
        res[0,:] = self.get_results()
        
        simulator.init()
        for i in range(1,nb_steps):
            self.time = self.time_step*float(i)
            self.simulator.loop_until(self.time)
            self.get_observations()
            self.step()
            res[i,:] = self.get_results()
        return res
        
    def step(self):
        self.prediction_step()
        # Perform a correction step
        self.correction_step()
        U, s, Vh = svd(self.Kk)
        print s

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
dt_simulator = 0.01 # s

# Period of the observations
dt_kalman = 0.1 # s

# Definition of the state space equations
# and initialization of the kalman filter
size_observations = 2
size_state = 4
size_control = size_state

# --- main
if __name__ == '__main__':
    np.random.seed()
    evolution_noise = 1e-2
    observation_noise = 1.0
    time_limit = 5

    simulator = Simulator([dt_simulator, g, m], [x0, y0, dx0, dy0])
    filter = KalmanFilter([dt_kalman, g, m], size_state, size_control, size_observations, simulator, evolution_noise, observation_noise)
    filter.init_state_from_array([0,5,0,0], 1.0)

    res_kalman = filter.simulation(time_limit)
    res_trajectory = simulator.gen_trajectory(time_limit)
    
    # Printing the result
    fig = plt.figure(figsize=(10,10))
    ax1 = fig.add_subplot(111)
    ax1.set_aspect('equal')

    # Plotting the estimated state
    ax1.plot(res_kalman[:,1], res_kalman[:,2],linewidth=2)
    # Plotting the observations
    ax1.plot(res_kalman[:,9], res_kalman[:,10],'o')    
    # Plotting the real state
    ax1.plot(res_trajectory[:,1], res_trajectory[:,2],linewidth=2)

    # print res_kalman.shape[0]
    for i in range(res_kalman.shape[0]):
        # We put an ellipse showing the uncertainty on the position
        e = Ellipse(xy=[res_kalman[i,1],res_kalman[i,2]], width=res_kalman[i,5], height=res_kalman[i,6], angle=0.0)
        ax1.add_artist(e)
        e.set_clip_box(ax1.bbox)
        e.set_alpha(0.8)
        e.set_facecolor([0.0,0.0,0.8])

    plt.xlabel("x position",fontsize=28)
    plt.ylabel("y position",fontsize=28)
    plt.title("Ball position",fontsize=28)
    
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend(['Estimation','Observation', 'Real'])
    
    #plt.savefig('KF-ex1.pdf',dpi=300)

    plt.show()




