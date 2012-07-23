from __future__ import print_function

import dynamics






from pylab import *



def zero_accel(x, tt):
    return zeros(6)

def coupled_oscillators_accel(x):
    aa = zeros(6)
    k = 0.5
    kc = 0.1 ## Coupling spring

    aa[0] = kc * x[1] - (k+kc) * x[0]
    aa[1] = kc * x[0] - (k+kc) * x[1]
    return aa

def gravity_accel(x):
    aa = zeros(6)
    gmm = 100.0
    dd = x[:2]
    aa[:2] = - (gmm / norm(dd)**3) * dd
    return aa

def pid_controler(x):
    
    aa = zeros(6)
    c1 = 4.0
    c2 = 4.0

    aa[:3] = -c1 * x[:3] - c2 * x[3:6]
    return aa
    



if __name__ == '__main__':
    set_printoptions(precision=3)

    sim = dynamics.Simulator()

    dt = 1e-3
    # T = 2*pi*dt ## Faster than that strange things happen.
    # T = 100

    x = zeros(13)
    x[0] = 10.0
    x[4] = 4.5
    x[6] = 1.0
    # x[10] = 2*pi/T

    y = zeros(13)


    Nt = 10000
    out = zeros((Nt+1, 13))
    out[0] = x
    for tt in range(Nt):
        # sim.simulation_step(y, x, dt, zero_accel, tt)
        # sim.simulation_step(y, x, dt, gravity_accel)
        # sim.simulation_step(y, x, dt, coupled_oscillators_accel)
        sim.simulation_step(y, x, dt, pid_controler)
        x[:] = copy(y)

        out[tt+1] = x


    vtime = mgrid[:Nt+1] * dt


    ion()

    # figure(1)
    # plot(out[:,6], out[:,7], '-', lw=1)
    # axis('equal')
    # grid()

    # figure(2)
    # title('Angular acceleration')
    
    # # plot(vtime, out[:,[6,7]], '-')
    # plot(vtime, out[:,7], '-')
    # plot(vtime, out[:,6], '-') 

    # plot(vtime, sin(vtime*2*pi/T), 'r--')
    # xlabel('Time')
    # ylabel('Quaternion params')
    # ylim(-1,1)
    # grid()

    # twinx()
    # plot(vtime, out[:,10], 'g-')
    # ylabel('Angular velocity')
    # ylim(0,3.0)





    figure(1, figsize=(6.4,8))
    suptitle('Critically-damped system')
    subplot(2,1,1)
    title('Track')
    plot(out[:,0], out[:,1], '-', lw=1)
    plot(0,0,'ks')
    axis('equal')
    grid()

    subplot(2,1,2)
    title('Individual parameters')
    plot(vtime, out[:,0], 'b-')
    plot(vtime, out[:,1], 'b-') 
    xlabel('Time')
    ylabel('Position')
    ylim(-10,10)
    twinx()
    plot(vtime, out[:,3], 'r-')
    plot(vtime, out[:,4], 'r-')
    ylim(-15,15)
    ylabel('Velocity')
    grid()
