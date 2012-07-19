

from __future__ import print_function
from pylab import *

from simple_kalman import *




def cal_acc_control_rocket(x, time, tt):
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

    for t1,t2,c1,c2 in tt:
        if time >= t1 and time < t2:
            k1 = c1*20.0
            k2 = c2*10.0

    accel[:3] = k1*R
    accel[4] = k2

    return accel








if __name__ == '__main__':

    kk = Kalman6DOF()

    kk.state[0] = -10.0
    kk.state[1] = 0.0
    kk.state[2] = 0.0

    kk.state[3] = 0.0
    kk.state[4] = 0.0
    kk.state[5] = 0.0

    ww0 = 0.0
    kk.state[6] = cos(ww0)
    kk.state[8] = -sin(ww0)

    # kk.state[11] = -0.1

    dt = 0.02

    Nt = 200
    out = zeros((Nt+1,13))

    ## Table with the activation times

    # trans_list = array([
    #         [  0, 10, 1, 0.0],
    #         [ 50, 56, 0,-1],
    #         [100,106, 0, 1],
    #         [110,130, 1, 0],
    #         ])

    trans_list = array([
            [  0, 10, 1, 0.0],
            [ 14, 30, 0,-1],
            [ 30, 46, 0, 1],
            [ 50, 70, 1, 0],

            [ 74, 90, 0,-1],
            [ 90,106, 0, 1],

            [110,131, 1, 0],

            [134,145, 0, 1],
            [145,156, 0,-1],
            [156,165,-1, 0],

            ])
    


    out[0] = kk.state ## Store initial state
    for k in mgrid[:Nt]:
        kk.predict_state_simulation(dt, cal_acc_control_rocket, k, trans_list)
        out[k+1] = kk.state ## Store next state

    ## Array with time values
    tt = dt * mgrid[:Nt+1]




    
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
        # plot([out[k,0],out[k,0]+ss*R[0,0]], [out[k,2],out[k,2]+ss*R[0,2]], 'k-' )
        plot([out[k,0],out[k,0]+ss*R[2,0]], [out[k,2],out[k,2]+ss*R[2,2]], 'k-' )
        plot(out[k,0], out[k,2], 'k.' )

    grid()

    # axis([-12,2,-1,5])
    xlim(-12,2)
    ylim(-1,5)


    xlabel('x position')
    ylabel('z position')
    title('Track')
    #legend(['Spaceship', 'Station'], 'lower left', ncol=1)




    subplot(2,1,2)
    title('Parameters on time')

    xlabel('Time')
    ylabel('Position and velocity')
    l1=plot(tt,out[:,0], 'b-')[0]
    plot(tt,out[:,2], 'b-')

    l2=plot(tt,out[:,3], 'r-')[0]
    plot(tt,out[:,5], 'r-')

    legend([l1,l2], ['Position','Velocity'], loc='upper left', ncol=1 )

    grid()



    # subplot(2,1,2)
    # title('Error')
    
    # xlabel('Time')
    # ylabel('Distance')
    # semilogy(tt,sqrt(((out[:,:3])**2).sum(1)), 'b-')[0]

    # ylim(1e-5,1e1)
    # grid()








