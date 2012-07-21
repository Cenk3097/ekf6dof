from __future__ import print_function

import dynamics






from pylab import *




def zero_accel(x):
    # aa = zeros(6)
    # aa[1] = 1.0
    # return aa
    return random(6)-0.5

if __name__ == '__main__':
    set_printoptions(precision=3)

    
    sim = dynamics.Simulator()

    x = zeros(13)
    x[3] = 1.0
    x[6] = 1.0
    x[10] = 1.0
    
    dt = 10.0

    y = zeros(13)


    Nt = 10000
    out = zeros((Nt, 2))
    print(x)
    for tt in range(Nt):

        sim.simulation_step(y, x, dt, zero_accel)
        print(y)
        x = y[:]

        out[tt, :2] = x[3:5]


    ion()

    plot(out[:,0], out[:,1], '-')

    axis('equal')
    grid()
