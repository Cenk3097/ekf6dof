

from __future__ import print_function
from pylab import *

from simple_kalman import *

import heapq




def cal_acc_control_rocket(x, tt):
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

    k1 = tt[0]*20.0
    k2 = tt[1]*10.0

    accel[:3] = k1*R
    accel[4] = k2

    return accel


def heuristics(x):

    return (x[0]**2 + x[2]**2 +
            x[3]**2 + x[5]**2 +
            (np.abs(x[6])-1)**2 + x[8]**2 +
            x[11]**2 )





def quantize(xx):
    xr = 1000
    vr = 2.5
    ar = 3.5
    wr = 3.5
    Q = array([ xr, xr, xr,
                vr, vr, vr,
                ar,ar,ar,ar,
                wr,wr,wr,])
    # return tuple(np.array(np.round(xx*Q), dtype=int16))
    return tuple(np.array(np.round(xx*Q), dtype=int16)[[0,2,3,5,6,8,11]])

    



if __name__ == '__main__':
    set_printoptions(precision=3)

    kk = Kalman6DOF()

    dt = 0.02

    Nt = 100

    actions = array([[-1, 0],
                     [ 0,-1],
                     [ 0, 0],
                     [ 0, 1],
                     [ 1, 0],
                     ])

    initial_state = zeros(13)
    initial_state[6] = -1.0
    initial_state[0] = 10.0

    final_state = zeros(13)
    final_state[6] = -1.0

    tovisit_states = []
    visited_states = set()

    heapq.heappush(tovisit_states,
                   (heuristics(initial_state),
                    tuple(initial_state))
                   )


    k=0
    while True:
        hh, so = heapq.heappop(tovisit_states)
        print(hh)
        so = array(so)

        visited_states.add(tuple(so))

        for aa in actions:
            kk.state = so
            kk.predict_state_simulation(dt, cal_acc_control_rocket, aa)
            
            sd = kk.state


            if tuple(sd) in visited_states:
                continue
 
            heapq.heappush(tovisit_states,
                           (heuristics(sd),
                            tuple(sd))
                           )
            k+=1
