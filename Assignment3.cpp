#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

    // declare variables
    double m, k, x, v, t_max, dt, t, a;
    vector<double> t_list, x_list, v_list;

    // mass, spring constant, initial position and velocity
    m = 1;
    k = 1;
    x = 0;
    v = 1;

    // simulation time and timestep
    t_max = 100;
    dt = 0.1;

    // Verlet integration for x
    //intitalize counter
    int i = 0;
    for (t = 0; t <= t_max; t = t + dt) {
        x_list.push_back(x);
        t_list.push_back(t);
        a = -k * x / m;
        if (i == 0) {
            x = x + dt * v;
        }
        else {
            x = 2 * x_list[i] - x_list[i - 1] + pow(dt,2) * a;
        }
       
        // append current state to trajectories
        // increment internal counter
        i++;
    }

    //Verlet Integration for v
   
    for (i = 0; i <= t_list.size(); i++) {
        v_list.push_back(v);
        if (i == 0){
            v = v + dt * a;
        }
        else {
            v = 1 / (2 * dt) * (x_list[i + 1] - x_list[i - 1]);
        }

    }


    // Write the trajectories to file
    ofstream fout;
    fout.open("trajectories.txt");
    if (fout) { // file opened successfully
        for (int i = 0; i < t_list.size(); i = i + 1) {
            fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
        }
    }
    else { // file did not open successfully
        cout << "Could not open trajectory file for writing" << endl;
    }

    /* The file can be loaded and visualised in Python as follows:

    import numpy as np
    import matplotlib.pyplot as plt
    results = np.loadtxt('trajectories.txt')
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(results[:, 0], results[:, 1], label='x (m)')
    plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
    plt.legend()
    plt.show()

    */
}
