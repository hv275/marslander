// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.
#include "lander.h"
#include <math.h>
#include <vector>
void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
    //as of 2/8/2020, adjusted to start after the thruster have been fired once
{   //initialise constants
    float K_h, K_p,e,P_o;
    //adjust by trial and error
    //K_h adjusts how long we wait until the thrusters kick/how high acceleration we want
    K_h = 0.03;
    K_p = 0.5;
    e = -(0.5 + K_h * alt(position) + velocity * position.norm());
    P_o = K_p * e;
    float Delta = 0.05;
    if (P_o < -Delta) {
        throttle = 0;
    }
    else if (-Delta < P_o < 1 - Delta) {
        throttle = Delta + P_o;
    }
    else {
        throttle = 1;
    }

    //following code outputs raw data for investigation and visualisation (Used MATLAB cause quicker)
    ofstream fout;
    fout.open("trajectories.txt", std::ofstream::app);
    if (fout) { // file opened successfully
        fout << alt(position) << ' ' << velocity * position.norm() << endl;
    }
    else { // file did not open successfully
        cout << "Could not open trajectory file for writing" << endl;
    }
}

//functions for acceleration calculations

vector3d gravacc(vector3d pos){

    return - pos.norm()*(GRAVITY * MARS_MASS / pos.abs2());
}

vector3d dragacc(vector3d vel,vector3d pos,  parachute_status_t chute) {
    if (chute == DEPLOYED)
        //assume the diameter of the lander to be 1m and projected area of parachute as negligible for lack of better info
        //pi is taken from maths.h
        return - vel.norm() * (0.5 * (atmospheric_density(pos) * (DRAG_COEF_LANDER + DRAG_COEF_CHUTE) * (M_PI * pow(LANDER_SIZE, 2)+5*2*LANDER_SIZE) * vel.abs2()))/ (UNLOADED_LANDER_MASS + FUEL_DENSITY * fuel);
    else {
    //may need to adjust some constants later to account for the area of the chute
        return - vel.norm() * (0.5 * (atmospheric_density(pos) * DRAG_COEF_LANDER * (M_PI * pow(LANDER_SIZE, 2)) * vel.abs2()))/(UNLOADED_LANDER_MASS+FUEL_DENSITY*fuel);
    }
}

double alt(vector3d pos) {
    return pos.abs() - MARS_RADIUS;
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
  //Note thrust_wrt_world takes no arguments and returns the the current thrust, defined in lander_graphics.cpp
  //my approach avoids using arrays to simplify the algorithm but it will cost accuracy for the approximation of velocity
  //NOTE: looping is already built into the code
    static vector3d last_pos, last_pos2;
    if (simulation_time <= 0.1) {
        last_pos = position;
        position = position + velocity * delta_t;
        velocity = velocity + delta_t * (gravacc(position) + dragacc(velocity, position, parachute_status) + thrust_wrt_world()/( UNLOADED_LANDER_MASS + FUEL_DENSITY * fuel));
        last_pos2 = last_pos;
    }
    else {
        last_pos = position;
        position = 2 * position - last_pos2 + pow(delta_t, 2) * (gravacc(position) + dragacc(velocity, position, parachute_status) + thrust_wrt_world()/( UNLOADED_LANDER_MASS + FUEL_DENSITY * fuel));
        velocity = (position - last_pos)/delta_t;
        last_pos2 = last_pos;
    }

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
      // an areostationary orbit
      position = vector3d((MARS_RADIUS + 17032000), 0.0,0.0);
      velocity = vector3d(0.0,1427.7,0.0);
      orientation = vector3d(0.0, 0.0, 90.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = true;
      autopilot_enabled = false;
      break;
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
