

from __future__ import print_function
from pylab import *

from simple_kalman import *




def cal_acc_const(x):
    ## Spring-mass system
    acc = -10.0
    return array([acc,0,0.0,0,0,0.0])

def cal_acc_hooke(x):
    ## Spring-mass system
    hooke = 20.0
    return array([-hooke*x[0],0,0.0,0,0,0.0])

def cal_acc_gravit(x):
    ## Spring-mass system
    gmm = 20.0
    aa = zeros(6)
    dd = x[:2]
    aa[:2] = -gmm * dd/norm(dd)/dot(dd,dd)

    return aa






if __name__ == '__main__':

    kk = Kalman6DOF()


    ## Projectile
    # kk.state[0] = 0.0
    # kk.state[3] = 10.0

    ## Spring-mass
    # kk.state[0] = -10.0
    # kk.state[3] = 0.0

    ## Comet
    kk.state[0] = -10.0
    kk.state[3] = 0.0
    kk.state[4] = 0.5

    dt = 0.01

    Nt = 4000
    out = zeros((Nt+1,13))

    out[0] = kk.state ## Store initial state
    for k in mgrid[:Nt]:

        # kk.predict_state_simulation(dt, cal_acc_const)
        # kk.predict_state_simulation(dt, cal_acc_hooke)
        kk.predict_state_simulation(dt, cal_acc_gravit)

        out[k+1] = kk.state ## Store next state

    ## Array with time values
    tt = dt * mgrid[:Nt+1]





    ion()


    ## Projectile
    # figure(1, figsize=(6.4,4.8))
    # suptitle('Dynamics simulation')
    # xlabel('Time')
    # ylabel('Position and velocity')
    # title('Projectile demo')

    # plot(tt, out[:,[0,3]], '-', ms=7,mew=1.2, lw=2)

    # legend(['Position','Velocity'], loc='lower left', ncol=2 )

    # grid()


    ## Spring-mass
    # figure(1, figsize=(6.4,9.6))
    # suptitle('Dynamics simulation')

    # hooke = 20.0
    # ww = sqrt(hooke)
    # # px = sin(ww*tt)/ww
    # # pv = cos(ww*tt)
    # px = -10*cos(ww*tt)
    # pv = 10*ww*sin(ww*tt)

    
    # subplot(2,1,1)
    # xlabel('Time')
    # ylabel('Position and velocity')
    # title('Spring-mass demo')

    # plot(tt, out[:,[0,3]], '-', ms=7,mew=1.2, lw=2)
    # plot(tt, px, 'r--')
    # plot(tt, pv, 'r--')

    # grid()


    # subplot(2,1,2)
    # xlabel('Time')
    # ylabel('Position and velocity errors')
    # title('Error')
    # pp = c_[px,pv]
    # plot(tt, out[:,[0,3]]-pp, '-', ms=7,mew=1.2, lw=2)

    # ylim(-.1,.1)
    # grid()




    ## Comet
    figure(1, figsize=(6.4,9.6))
    suptitle('Dynamics simulation')

    subplot(2,1,1)
    plot(out[0,0], out[0,1], 'bo-')
    plot(0,0,'ks')
    plot(out[:,0], out[:,1], '-', ms=7,mew=1.2, lw=1)
    axis('equal')
    xlim(-11,2)

    grid()

    xlabel('x position')
    ylabel('y position')
    title('Gravitational force')
    legend(['Comet track', 'Star'], 'lower left', ncol=1)


    subplot(2,1,2)
    title('Parameters on time')
    
    xlabel('Time')
    ylabel('Position and velocity')
    l1=plot(tt,out[:,0], 'b-')[0]
    plot(tt,out[:,1], 'b-')

    # grid()
    # aa=axis()

    l2=plot(tt,out[:,3], 'r-')[0]
    plot(tt,out[:,4], 'r-')

    legend([l1,l2], ['Position','Velocity'], loc='upper left', ncol=2 )

    ylim(-10,10)

    grid()
    

