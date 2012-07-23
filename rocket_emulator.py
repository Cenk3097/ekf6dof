from __future__ import print_function

import dynamics

from pylab import *


def matrix_from_quaternion(q):
    a,b,c,d = q
    return np.array([ [(a*a+b*b-c*c-d*d), (2*b*c-2*a*d),     (2*b*d+2*a*c)     ],
                      [(2*b*c+2*a*d),     (a*a-b*b+c*c-d*d), (2*c*d-2*a*b)     ],
                      [(2*b*d-2*a*c),     (2*c*d+2*a*b),     (a*a-b*b-c*c+d*d)] ] )


def rocket_controller(x, time, tt):
    ## In this case we have a spaceship that is guided by three
    ## thrusters. We can control the roll and the pitch, not yaw, and
    ## we can also accelerate forward or backwards.

    ## The target is to "park" the ship at the origin
    ## The goal state is all zeros.
    desired = zeros(13)
    desired[6] = 1.0
                        
    ## Simply return the error. Suppose we can accelerate to any direction.
    accel = zeros(6)

    ## Calculate the "forward" direction of the ship.
    R = matrix_from_quaternion(x[6:10])[2]

    k1 = 0.0
    k2 = 0.0

    for t1,t2,com in tt:
        # if time >= t1 and time < t2:
            if com == 1:
                k1 = 20.0
                k2 = 0.0
            elif com == 2:
                k1 = -20.0
                k2 = 0.0
            elif com == 3:
                k1 = 0.0
                k2 = 10.0
            elif com == 4:
                k1 = 0.0
                k2 = -10.0

    


    accel[:3] = k1*R
    accel[4] = k2

    return accel



if __name__ == '__main__':
    set_printoptions(precision=3)

    sim = dynamics.Simulator()

    dt = 1e-2
    # T = 2*pi*dt ## Faster than that strange things happen.
    # T = 100

    x = zeros(13)
    x[0] = 10.0
    x[4] = 4.5
    # x[6] = 1.0
    x[6] = sqrt(.7)
    x[8] = sqrt(.7)

    # x[10] = 2*pi/T

    y = zeros(13)


    tab = array([
            [0, 20, 3],
            [20, 40, 4],
            [40, 60, 1],
            [60, 80, 3],
            [80, 100, 4],
            [140, 160, 1],
            ])


    Nt = 10000
    out = zeros((Nt+1, 13))
    out[0] = x
    for tt in range(Nt):
        # sim.simulation_step(y, x, dt, zero_accel, tt)
        # sim.simulation_step(y, x, dt, gravity_accel)
        # sim.simulation_step(y, x, dt, coupled_oscillators_accel)
        # sim.simulation_step(y, x, dt, pid_controller)
        sim.simulation_step(y, x, dt, rocket_controller, tt, tab)
        x[:] = copy(y)

        out[tt+1] = x


    time = mgrid[:Nt+1] * dt


    ion()


    ## Plot stuff
    figure(1, figsize=(8,8))
    suptitle('Parking a spaceship')

    subplot(2,1,1)
    plot(out[0,0], out[0,2], 'bo-')
    plot(0,0,'ks')
    plot(out[:,0], out[:,2], '-', ms=7,mew=1.2, lw=1)
    axis('equal')


    ## Plot the ship reference frame at a few points
    ss = -0.5
    for k in mgrid[:Nt+1:10]:
        R = matrix_from_quaternion(out[k,6:10])
        plot([out[k,0],out[k,0]+ss*R[2,0]], [out[k,2],out[k,2]+ss*R[2,2]], 'k-' )
        plot(out[k,0], out[k,2], 'k.' )


    grid()

    # axis([-12,2,-1,5])
    xlim(-2,12)
    ylim(-1,5)


    xlabel('x position')
    ylabel('z position')
    title('Track')
    #legend(['Spaceship', 'Station'], 'lower left', ncol=1)


    subplot(2,1,2)
    title('Parameters on time')

    xlabel('Time')
    ylabel('Position and velocity')
    l1=plot(time,out[:,0], 'b-')[0]
    plot(time,out[:,2], 'b-')

    l2=plot(time,out[:,3], 'r-')[0]
    plot(time,out[:,5], 'r-')

    legend([l1,l2], ['Position','Velocity'], loc='upper left', ncol=1 )

    grid()
