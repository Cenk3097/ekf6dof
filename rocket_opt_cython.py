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
    x[10] = sin(2*pi/20000.0)
    
    dt = 0.2


    y = zeros(13)


    Nt = 100001
    out = zeros((Nt+1, 13))
    out[0] = x
    for tt in range(Nt):
        sim.simulation_step(y, x, dt, zero_accel)
        x[:] = copy(y)

        out[tt+1] = x


    vtime = mgrid[:Nt+1] * dt


    ion()

    figure(1)
    plot(out[:,6], out[:,7], '-', lw=1)
    axis('equal')
    grid()

    figure(2)
    plot(vtime, out[:,[6,7]], '-')
    grid()
