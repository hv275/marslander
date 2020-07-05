# -*- coding: utf-8 -*-
"""
Created on Wed Jul  1 22:29:55 2020

@author: glebv
"""

#will keep integration functions here
#euler integral
def Euler(position,velocity,t_array,dt):
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