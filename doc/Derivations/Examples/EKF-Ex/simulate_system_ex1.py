#!/usr/bin/env python

''' 
Simulation of the dynamical system 
y'(t) = 2.0/ (1+ exp(-1*(z-1))) - 1.0
z'(t) = -5*y
y(0) = 1.0
z(0) = 0.0
with a RK 4
'''

from numpy import *
import matplotlib.pyplot as plt

class Euler:
    
    def __init__(self, y0, z0, dt, dy_equation, dz_equation):
        self.y0 = y0
        self.z0 = z0
        self.dt = dt
        self.y_values = array([y0])
        self.z_values = array([z0])
        self.t_values = array([0.0])
        self.t = 0
        self.dy_equation = eval('lambda t,y,z: '+ dy_equation_str)
        self.dz_equation = eval('lambda t,y,z: '+ dz_equation_str)

    def update_data(self):
        k1_y = self.dy_equation(self.t_values[-1],self.y_values[-1],self.z_values[-1])
        k1_z = self.dz_equation(self.t_values[-1],self.y_values[-1],self.z_values[-1])
         
        new_y_value = self.y_values[-1] + self.dt*self.dy_equation(self.t_values[-1],self.y_values[-1],self.z_values[-1])
        new_z_value = self.z_values[-1] + self.dt*self.dz_equation(self.t_values[-1],self.y_values[-1],self.z_values[-1])
        
        self.y_values = append(self.y_values, new_y_value)
        self.z_values = append(self.z_values, new_z_value)            
        self.t_values = append(self.t_values, self.t_values[-1] + self.dt)

    def loop_until(self, time_limit):
        print "Starting simulation with dt = ", self.dt
        while(self.t_values[-1] < time_limit):
            self.update_data()
        print "Simulation finished"

class RK4:
    
    def __init__(self, y0, z0, dt, dy_equation, dz_equation):
        self.y0 = y0
        self.z0 = z0
        self.dt = dt
        self.y_values = array([y0])
        self.z_values = array([z0])
        self.t_values = array([0.0])
        self.t = 0
        self.dy_equation = eval('lambda t,y,z: '+ dy_equation_str)
        self.dz_equation = eval('lambda t,y,z: '+ dz_equation_str)

    def update_data(self):
        k1_y = self.dy_equation(self.t_values[-1],self.y_values[-1],self.z_values[-1])
        k1_z = self.dz_equation(self.t_values[-1],self.y_values[-1],self.z_values[-1])
        
        k2_y =self. dy_equation(self.t_values[-1]+0.5 * self.dt, self.y_values[-1] + 0.5 * self.dt * k1_y, self.z_values[-1] + 0.5 * self.dt * k1_z)
        k2_z = self.dz_equation(self.t_values[-1]+0.5 * self.dt, self.y_values[-1] + 0.5 * self.dt * k1_y, self.z_values[-1] + 0.5 * self.dt * k1_z)
        
        k3_y = self.dy_equation(self.t_values[-1]+0.5 * self.dt, self.y_values[-1] + 0.5 * self.dt * k2_y, self.z_values[-1] + 0.5 * self.dt * k2_z)
        k3_z = self.dz_equation(self.t_values[-1]+0.5 * self.dt, self.y_values[-1] + 0.5 * self.dt * k2_y, self.z_values[-1] + 0.5 * self.dt * k2_z)
        
        k4_y = self.dy_equation(self.t_values[-1]+self.dt, self.y_values[-1] + self.dt * k3_y, self.z_values[-1] + self.dt * k3_z)
        k4_z = self.dz_equation(self.t_values[-1]+self.dt, self.y_values[-1] + self.dt * k3_y, self.z_values[-1] + self.dt * k3_z)
        
        new_y_value = self.y_values[-1] + self.dt/6.0 * (k1_y + 2.0*k2_y + 2.0 * k3_y + k4_y)
        new_z_value = self.z_values[-1] + self.dt/6.0 * (k1_z + 2.0*k2_z + 2.0 * k3_z + k4_z)
        
        self.y_values = append(self.y_values, new_y_value)
        self.z_values = append(self.z_values, new_z_value)            
        self.t_values = append(self.t_values, self.t_values[-1] + self.dt)

    def loop_until(self, time_limit):
        while(self.t_values[-1] < time_limit):
            self.update_data()
        print "Simulation finished"

# --- main
if __name__ == '__main__':
    y0 = 1
    z0 = 0
    dt = 0.0005
    dy_equation_str = "2.0 / (1.0 + exp(-z+1)) - 1.0"
    dz_equation_str = "-0.1 * y"
    #simulator = RK4(y0, z0, 20.0*dt, dy_equation_str, dz_equation_str)
    #simulator.loop_until(10.0)

    simulator_euler = Euler(y0, z0, dt, dy_equation_str, dz_equation_str)
    simulator_euler.loop_until(30.0)
    
    fig = plt.figure(figsize=(10,10))
    ax1 = fig.add_subplot(111)
    #ax1.set_aspect('equal')
    #ax1.plot(simulator.t_values, simulator.y_values)
    #ax1.plot(simulator.t_values, simulator.z_values)

    ax1.plot(simulator_euler.t_values, simulator_euler.y_values)
    ax1.plot(simulator_euler.t_values, simulator_euler.z_values) 
   
    plt.xlabel("Time",fontsize=28)
    plt.ylabel("Value",fontsize=28)
    plt.title("Dynamical system",fontsize=28)
    
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend(['y(t)','z(t)'])
    plt.savefig('simulation_ex1.pdf',dpi=300)

    plt.show()
