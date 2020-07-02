# -*- coding: utf-8 -*-
"""
Created on Wed Jul  1 22:03:48 2020

@author: glebv
"""

import numpy as np
#get value of G
from scipy.constants import G
import matplotlib.pyplot as plt

#initialise coordinate and velocity arrays
#position expressed as an altitude
altitude = 100
position = np.array([1,0,0])*(3389.5e3 + altitude)
velocity = np.zeros(3)
#create time array
t_max = 100
dt = 0.1
t_list = np.arange(0,t_max,dt)

#acceleration vector as a function of distance
def a(position):
    r= np.linalg.norm(position)
    return -(G * 6.42e23)/r**2 * (position/r)

acc = a(position)

#fuction for euler integral
def Euler(position,velocity,t_array):
    pos_list = []
    vel_list = []
    for t in t_array:
        # append current state to trajectories
        pos_list.append(position)
        vel_list.append(velocity)
    
        # calculate new position and velocity
        position = position + dt * velocity
        velocity = velocity + dt * a(position)
    return pos_list,vel_list

def Verlet(position,velocity,t_array):
    pos_list = [position]
    vel_list = [velocity]
    #integrate to find position
    for i in range(len(t_array)-1):
        if i == 0:
            position = position + dt * velocity
        else:
            position= 2*pos_list[i]-pos_list[i-1]+dt**2*a(position)
        pos_list.append(position)
    #integrate for velocity
    for i in range(len(t_array)-1):
        if i == 0:
            velocity = velocity + dt * a(position)
        else:
            velocity= 1/(2*dt)*(pos_list[i+1]-pos_list[i-1])
        vel_list.append(velocity)
    return pos_list, vel_list
positions, velocities = Verlet(position,velocity,t_list)
positions = np.array(positions)


