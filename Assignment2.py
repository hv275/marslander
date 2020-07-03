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
altitude = 1380e3
r_planet = 3389.5e3
position = np.array([1,0,0])*(r_planet + altitude)
velocity = np.zeros(3)
#create time array
t_max = 10000
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

#function for verlet integrak
def Verlet(position,velocity,t_array):
    pos_list = [position]
    vel_list = [velocity]
    #integrate to find position
    #the -1 in length assures that array is the same length aa t_array
    for i in range(len(t_array)-1):
        #for first element, we use euler inegration as there is no previous element
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
        #make np arrays
    pos_list = np.array(pos_list)
    vel_list = np.array(vel_list)
    return pos_list, vel_list



positions, velocities = Verlet(position,velocity,t_list)


#set up an altitude calculator
altitudes = []
for i in positions:
    alt = np.linalg.norm(i)-r_planet
    if alt >= 0:
        altitudes.append(alt)
    else:
        #as soon as we hit the surface, finish and fill the rest with 0
        altitudes.append(0)
        break
    
altitudes = np.array(altitudes)
#fill the rest with zeros as we have crashed 
altitudes.resize(len(t_list))  
#plot scenario 1
plt.figure(1)
plt.xlabel('time (s)')
plt.ylabel('altitude (m)')
plt.title("Scenario 1: Fall from constant height")
plt.grid()
plt.plot(t_list,altitudes)
plt.show()

#set up initial conditions for scenario 2
#start with simple conditions
#could write an algorithm to make velocity perpendicular but keeping it simple
position = np.array([1,0,0])*(r_planet + altitude)
#find needed velocity for this orbit
velocity = np.array([0,1,0])*(np.sqrt(G*6.42e23/np.linalg.norm(position)))
#integrate again
positions, velocities = Verlet(position,velocity,t_list)
#for ease of plotting, transpose
positions = np.transpose(positions)
#plot the trajectory
plt.figure(2)
plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.title("Scenario 2: Circular Orbit")
plt.plot(positions[1],positions[0])
plt.show()


#set up initial conditions for scenario 3
position = np.array([1,0,0])*(r_planet + altitude)
#find needed velocity for this orbit
velocity = np.array([0,1,0])*(1000)
#integrate again
positions, velocities = Verlet(position,velocity,t_list)
#for ease of plotting, transpose
positions = np.transpose(positions)
#plot the trajectory
plt.figure(3)
plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.title("Scenario 3: Elliptical Orbit")
plt.plot(positions[0],positions[1])
plt.show()


#set up initial conditions for scenario 4
position = np.array([1,0,0])*(r_planet + altitude)
#find needed velocity for this orbit (use escape velocity)
velocity = np.array([0,1,0])*(np.sqrt(2*G*6.42e23/np.linalg.norm(position)))
#integrate again
positions, velocities = Verlet(position,velocity,t_list)
#for ease of plotting, transpose
positions = np.transpose(positions)
#plot the trajectory
plt.figure(4)
plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.title("Scenario 4: Hyperbolic Orbit")
plt.plot(positions[0],positions[1])
plt.show()


