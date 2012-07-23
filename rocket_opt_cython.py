from __future__ import print_function

import dynamics






from pylab import *




def rocket_controls(x, time, com):
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
    else:
        k1 = 0.0
        k2 = 0.0

    accel[:3] = k1*R
    accel[4] = k2

    return accel





def quantize(x, lowlim, parmrg):
    xx = x[[0,2,3,5,6,8,11]]
    xr = 1.0
    vr = 1.0
    ar = 3.0 * sign(xx[6])
    wr = 1.0
    Q = array([ xr, xr,
                vr, vr,
                ar, ar,
                wr,])

    qq = np.array(np.round(xx*Q), dtype=int16)

    out = 0
    for k in range(lowlim.shape[0]):
        vv = qq[k] - lowlim[k]
        if vv < 0 or vv > parmrg[k]:
            raise ValueError
        out *= parmrg[k]
        out += vv
    return out


if __name__ == '__main__':
    set_printoptions(precision=3)

    sim = dynamics.Simulator()

    dt = 1e-3
    Nt=10000


    target = zeros(13)
    target[6] = sqrt(.5)
    target[6] = sqrt(.5)


    out = zeros((Nt+1,13))
    for k in range(Nt):

        
        sim.simulation_step(y, x, dt, rocket_controls, k, action)
        x[:] = copy(y)

        out[k+1] = x


    vtime = mgrid[:Nt+1] * dt


    ion()
    plot(out[:,0], out[:,1], 'b-')
    axis('equal')
    grid()
    
