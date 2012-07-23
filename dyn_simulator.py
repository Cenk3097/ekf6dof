from __future__ import print_function

import dynamics






from pylab import *




def matrix_from_quaternion(q):
    a,b,c,d = q
    return np.array([ [(a*a+b*b-c*c-d*d), (2*b*c-2*a*d),     (2*b*d+2*a*c)     ],
                      [(2*b*c+2*a*d),     (a*a-b*b+c*c-d*d), (2*c*d-2*a*b)     ],
                      [(2*b*d-2*a*c),     (2*c*d+2*a*b),     (a*a-b*b-c*c+d*d)] ] )


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

def pid_controller(x):    
    aa = zeros(6)
    c1 = 4.0
    c2 = 1.0

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
    x[5] = 4.5
    x[6] = 1.0
    # x[10] = 2*pi/T

    y = zeros(13)


    tab = array([
            [0, 100, 1,0],
            [100, 200, 0,1],
            [110, 200, 1,0],

            ])


    Nt = 10000
    out = zeros((Nt+1, 13))
    out[0] = x
    for tt in range(Nt):
        # sim.simulation_step(y, x, dt, zero_accel, tt)
        # sim.simulation_step(y, x, dt, gravity_accel)
        # sim.simulation_step(y, x, dt, coupled_oscillators_accel)
        sim.simulation_step(y, x, dt, pid_controller)
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

    # figure(1, figsize=(6.4,8))
    # suptitle('Critically-damped system')
    # subplot(2,1,1)
    # title('Track')
    # plot(out[:,0], out[:,1], '-', lw=1)
    # plot(0,0,'ks')
    # axis('equal')
    # grid()

    # subplot(2,1,2)
    # title('Individual parameters')
    # plot(vtime, out[:,0], 'b-')
    # plot(vtime, out[:,1], 'b-') 
    # xlabel('Time')
    # ylabel('Position')
    # ylim(-10,10)
    # twinx()
    # plot(vtime, out[:,3], 'r-')
    # plot(vtime, out[:,4], 'r-')
    # ylim(-15,15)
    # ylabel('Velocity')
    # grid()



    ## Plot stuff
    figure(1, figsize=(8,8))
    suptitle('Parking a spaceship')

    subplot(2,1,1)
    plot(out[0,0], out[0,2], 'bo-')
    plot(0,0,'ks')
    plot(out[:,0], out[:,2], '-', ms=7,mew=1.2, lw=1)
    axis('equal')

    # axis([-12,2,-1,5])
    # xlim(-12,2)
    # ylim(-1,5)


    xlabel('x position')
    ylabel('z position')
    title('Track')
    #legend(['Spaceship', 'Station'], 'lower left', ncol=1)




    subplot(2,1,2)
    title('Parameters on time')

    xlabel('Time')
    ylabel('Position and velocity')
    l1=plot(vtime,out[:,0], 'b-')[0]
    plot(vtime,out[:,2], 'b-')

    l2=plot(vtime,out[:,3], 'r-')[0]
    plot(vtime,out[:,5], 'r-')

    legend([l1,l2], ['Position','Velocity'], loc='upper left', ncol=1 )

    grid()
