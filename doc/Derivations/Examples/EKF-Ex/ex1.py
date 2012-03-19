#!/usr/bin/env python

'''
Extended Kalman Filters : Example 1
We seek to estimate the state of a dynamical system which, at some times, is almost linear while becoming strongly non linear at others. This is an example to show that EKF does not perform well when the evolution function becomes non linear but then recovers good performances when we go back into a linear regime.
y'(t) = 2.0/ (1+ exp(-1*(z-1))) - 1.0
z'(t) = k*y
y(0) = 1.0
z(0) = 0.0
'''

from numpy import *
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

class EulerSimulator:
    
    def __init__(self, y0, z0, dt, dx_equation, dy_equation):
        self.y0 = y0
        self.z0 = z0
        self.dt = dt
        self.x_values = array([y0])
        self.y_values = array([z0])
        self.t_values = array([0.0])
        self.t = 0
        self.dx_equation = eval('lambda t,x,y: '+ dx_equation_str)
        self.dy_equation = eval('lambda t,x,y: '+ dy_equation_str)

    def init(self):
        self.x_values = array([y0])
        self.y_values = array([z0])
        self.t_values = array([0.0])
        self.t = 0

    def gen_trajectory(self, time_limit):
        self.init()
        self.loop_until(time_limit)
        data = transpose(reshape(append(append(self.t_values, self.x_values), self.y_values), (3,self.t_values.size)))
        return data

    def update_data(self):
        k1_y = self.dx_equation(self.t_values[-1],self.x_values[-1],self.y_values[-1])
        k1_z = self.dy_equation(self.t_values[-1],self.x_values[-1],self.y_values[-1])
         
        new_y_value = self.x_values[-1] + self.dt*self.dx_equation(self.t_values[-1],self.x_values[-1],self.y_values[-1])
        new_z_value = self.y_values[-1] + self.dt*self.dy_equation(self.t_values[-1],self.x_values[-1],self.y_values[-1])
        
        self.x_values = append(self.x_values, new_y_value)
        self.y_values = append(self.y_values, new_z_value)            
        self.t_values = append(self.t_values, self.t_values[-1] + self.dt)

    def loop_until(self, time_limit):
        while(self.t_values[-1] < time_limit):
            self.update_data()

    def get_state(self):
        return [self.x_values[-1], self.y_values[-1]]

    def get_observations(self):
        return matrix(array([self.x_values[-1], self.y_values[-1]])).T

class ExtendedKalmanFilter:
    '''Kalman filter for linear state space'''
    
    def __init__(self, time_step, size_state, size_observations, simulator, evolution_noise, observation_noise, evolution_function, evolution_jacobian , observation_function, observation_jacobian, is_noisy, noise_variance=0):
        self.size_state = size_state
        self.size_observations = size_observations
        self.dt = time_step

        self.evolution_function = evolution_function
        self.evolution_jacobian = evolution_jacobian
        self.observation_function = observation_function
        self.observation_jacobian = observation_jacobian

        self.Pk = matrix(zeros((size_state, size_state)))
        self.Kk = matrix(zeros((size_state, size_observations)))

        self.Qk = evolution_noise * matrix(identity(size_state))
        self.Rk = observation_noise * matrix(identity(size_observations))

        self.simulator = simulator

        self.is_noisy = is_noisy
        self.noise_variance = noise_variance
    
    def init_state_from_array(self, xk, initial_covariance):
        self.xk = matrix(xk).T
        self.Pk = initial_covariance * identity(self.size_state)

    def get_observations(self):
        self.yk = self.simulator.get_observations() + (random.normal(0.0,self.noise_variance,(self.size_observations,1)) if self.is_noisy else 0)
        
    def prediction_step(self):
        self.xkm = self.evolution_function(self.dt, self.xk)
        self.Fk = self.evolution_jacobian(self.dt, self.xk)
        self.Pkm = self.Fk * self.Pk * self.Fk.T + self.Qk
        #print "After predictio nstep : ", self.xkm

    def correction_step(self):
        self.Hk = self.observation_jacobian(self.dt, self.xkm)
        self.Kk = self.Pkm * self.Hk.T * (self.Hk * self.Pkm * self.Hk.T+self.Rk).I
        self.Pk = (matrix(identity(self.size_state)) - self.Kk * self.Hk) * self.Pkm
        self.xk = self.xkm + self.Kk * (self.yk - self.observation_function(self.dt,self.xkm))

    def simulation(self, time_limit):
        nb_steps = 2+int(time_limit/float(self.dt))
        res = zeros(( nb_steps, 1+2*self.size_state+self.size_observations+self.size_state))
        # Some initialization
        self.simulator.init()

        # Save the time, estimated state, diagonal elements of the covariance and true state
        index = 0
        res[0,index] = 0
        index+=1
        for j in range(self.size_state):
            res[0,index] = asarray(self.xk)[j]
            index+=1
        for j in range(self.size_state):
            res[0,index] = asarray(self.Pk)[j,j]
            index+=1
        for j in range(self.size_observations):
            res[0, index] = 0
            index+=1
        for j in range(self.size_state):
            res[0,index] = self.simulator.get_state()[j]
            index+=1
        
        # Perform a prediction step
        for i in range(1,nb_steps):
            t = self.dt*float(i)
            self.simulator.loop_until(t)
            self.get_observations()
            self.step()

            # Save the time, estimated state, diagonal elements of the covariance and true state
            index = 0
            res[i,index] = t
            index+=1
            for j in range(self.size_state):
                res[i,index] = asarray(self.xk)[j]
                index+=1
            # Keep track of the diagonal elements of Pk
            for j in range(self.size_state):
                res[i,index] = asarray(self.Pk)[j,j]
                index+=1
            for j in range(self.size_observations):
                res[i, index] = self.yk[j]
                index+=1
            for j in range(self.size_state):
                res[i,index] = self.simulator.get_state()[j]
                index+=1
        return res
        
    def step(self):
        self.prediction_step()
        # Perform a correction step
        self.correction_step()

#################################################################################
################# The only part you need to modify is below this line ###########
#################################################################################

# Definition of the state space equations
# and initialization of the kalman filter
def evolution_function(dt, state):
    x = asarray(state)[0]
    y = asarray(state)[1]
    res = matrix(array([x + dt * (2.0 / (1.0 + exp(-y+1)) - 1.0),y + dt * (factor_y * x)]))
    return res

def evolution_jacobian(dt, state):
    x = asarray(state)[0]
    y = asarray(state)[1]
    res =  matrix(array([1.0, 2.0 * dt * (1.0 - 1.0 / (1.0 + exp(-y+1))) ,
                         factor_y*dt, 1.0]).reshape((2,2)))
    return res

def observation_function(dt, state):
    return state

def observation_jacobian(dt, state):
    return matrix(identity(size_observations))

# --- main
if __name__ == '__main__':
    # Definition of the dynamical system and the parameters
    size_observations = 2
    size_state = 2
    factor_y = -0.1
    time_limit = 30
    
    y0 = 1
    z0 = 0
    dt = 0.0005
    dx_equation_str = "2.0 / (1.0 + exp(-y+1)) - 1.0"
    dy_equation_str = str(factor_y) + " * x"
    evolution_noise = 1e-2
    observation_noise = 1e-0

    noise_variance = 0.5
    is_noisy = True

    # Physical simulator
    simulator = EulerSimulator(y0, z0, dt, dx_equation_str, dy_equation_str)
    simulator.init()
    # Extended Kalman Filter
    filter = ExtendedKalmanFilter(1000*dt, size_state, size_observations, simulator, evolution_noise, observation_noise, evolution_function, evolution_jacobian , observation_function, observation_jacobian, is_noisy, noise_variance)
    
    # Initialization of the inital state estimate and associated covariance
    filter.init_state_from_array([0,0], 1.0)

    # Simulation of EKF
    res_kalman = filter.simulation(time_limit)

    # Simulator of the physical system with a small time step
    res_trajectory = simulator.gen_trajectory(time_limit)
    
    # Printing the result
    fig = plt.figure(figsize=(25,7))
    ax1 = fig.add_subplot(121)
    plt.xlabel("Value",fontsize=28)
    plt.ylabel("Time(s.)",fontsize=28)
    plt.title("Dynamical system",fontsize=28)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)

    ax2 = fig.add_subplot(122)
    plt.xlabel("Time(s.)",fontsize=28)
    plt.ylabel("Value",fontsize=28)
    plt.title("EKF estimates",fontsize=28)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)

    ax1.plot(res_trajectory[:,0], res_trajectory[:,1])
    ax1.plot(res_trajectory[:,0], res_trajectory[:,2])
    plt.legend(['x(t)','y(t)'])

    # Plotting the estimated state
    ax2.plot(res_kalman[:,0], res_kalman[:,5],'ro',markersize=3)
    ax2.plot(res_kalman[:,0], res_kalman[:,6],'bo',markersize=3)   
    ax2.plot(res_kalman[:,0], res_kalman[:,1])
    ax2.plot(res_kalman[:,0], res_kalman[:,2]) 
    plt.legend(['x(t)','y(t)'])

    #plt.savefig('EKF-ex1.pdf',dpi=300)

    plt.show()




