from __future__ import print_function

import dynamics






from pylab import *



def zero_accel(x, tt):
    aa = zeros(6)
    #aa[1] = 1.0
    #aa[3] = 1e-6

    # if tt == 4000:
    #     aa[3] = 50.0

    # if tt >= 2000 and tt < 3000:
    #     aa[3] = 5e-2
    # if tt >= 4000 and tt < 5000:
    #     aa[3] = 5e-2
    # if tt >= 6000 and tt < 7000:
    #     aa[3] = 5e-2
    # if tt >= 8000 and tt < 9000:
    #     aa[3] = 5e-2

    if tt >= 3000 and tt < 7000:
        aa[3] = 5e-2


    # aa = random(6)-0.5

    return aa


if __name__ == '__main__':
    set_printoptions(precision=3)

    
    sim = dynamics.Simulator()

    x = zeros(13)
    x[3] = 0.0
    x[6] = 1.0
    x[10] = sin(2*pi/20.0)
    
    dt = 0.01


    y = zeros(13)


    Nt = 10000
    out = zeros((Nt+1, 13))
    out[0] = x
    for tt in range(Nt):
        sim.simulation_step(y, x, dt, zero_accel, tt)
        x[:] = copy(y)

        out[tt+1] = x


    vtime = mgrid[:Nt+1] * dt


    ion()

    figure(1)
    plot(out[:,6], out[:,7], '-', lw=1)
    axis('equal')
    grid()

    figure(2)
    title('Angular acceleration')
    
    # plot(vtime, out[:,[6,7]], '-')
    plot(vtime, out[:,7], '-')
    plot(vtime, out[:,6], '-') 
    xlabel('Time')
    ylabel('Quaternion params')
    ylim(-1,1)
    grid()

    twinx()
    plot(vtime, out[:,10], 'g-')
    ylabel('Angular velocity')
    ylim(0,3.0)
