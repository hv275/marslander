# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 19:57:19 2020

@author: glebv
"""

# Note to reader, Euler method is unstable as fuck and will keep trying to explode for small values of dt
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
x0=0
v = 1
# simulation time, timestep and time
t_max = 100
#Verlet stable until one and begins blowing up after 1.5
dt = 0.001
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []
#empty lists for vertlet integrator
x_vert = [0]
v_vert = [1]
# Euler integration
for t in t_array:

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate new position and velocity
    a = -k * x / m
    x = x + dt * v
    v = v + dt * a
    
    
#reset initial conditions
x=0
v=1
# Verlet Intgration
for i in range(len(t_array)):
    #append current state
    
    #calculate new position and velocity
    a = -k * x_vert[i] / m
    if i == 0:
        x = x + dt * 1
    else:
        x= 2*x_vert[i]-x_vert[i-1]+dt**2*a
    x_vert.append(x)
    

#had to do it via two loops as v can only be claculated after x    
for i in range(len(t_array)):
    a = -k * x_vert[i] / m
    if i == 0:
        v = v + dt * a
    else:
        v= 1/(2*dt)*(x_vert[i+1]-x_vert[i-1])
    v_vert.append(v)
    
#get rid of over assigned value
x_vert.pop()
v_vert.pop()

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()

plt.figure(2)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_vert, label='x (m)')
plt.plot(t_array, v_vert, label='v (m/s)')
plt.legend()
plt.show()

