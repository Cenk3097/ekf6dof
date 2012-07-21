from __future__ import print_function

import dynamics






from pylab import *




def zero_accel(x):
    aa = zeros(6)
    # aa[1] = 1.0
    return aa
    # return random(6)-0.5

if __name__ == '__main__':
    set_printoptions(precision=3)

    
    sim = dynamics.Simulator()

    x = zeros(13)
    x[3] = 0.0
    x[6] = 1.0
    x[10] = 4.19
    
    dt = 0.001

    y = zeros(13)


    Nt = 10000
    out = zeros((Nt+1, 13))
    out[0] = x
    #print(x)
    for tt in range(Nt):

        sim.simulation_step(y, x, dt, zero_accel)
        #print(y)
        x = y[:]

        out[tt+1] = x


    vtime = mgrid[:Nt+1] * dt


    ion()

    figure(1)
    plot(out[:,6], out[:,7], '-+', lw=0.1, alpha=0.75)
    axis('equal')
    grid()

    figure(2)
    plot(vtime, out[:,[6,7]], '-')
    grid()
