

from __future__ import print_function
from pylab import *

from simple_kalman import *




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







def quantize(xx):
    xr = 100
    vr = 2.5
    ar = 3.5
    wr = 3.5
    Q = array([ xr, xr, xr,
                vr, vr, vr,
                ar,ar,ar,ar,
                wr,wr,wr,])
    # return tuple(np.array(np.round(xx*Q), dtype=int16))
    return tuple(np.array(np.round(xx*Q), dtype=int16)[[0,2,3,5,6,8,11]])


def out_of_bounds(qq):
    # if (qq[0] < -2 or qq[0] > 12 or
    #     qq[1] != 0               or
    #     qq[2] <  -1 or qq[2] > 5 or
    #     qq[3] <  -4 or qq[3] > 4 or
    #     qq[4] <  -4 or qq[4] > 4 or
    #     qq[5] <  -4 or qq[5] > 4 or
    #     qq[6] <  -3 or qq[6] > 3 or
    #     qq[7] <  -3 or qq[7] > 3 or
    #     qq[8] <  -3 or qq[8] > 3 or
    #     qq[9] <  -3 or qq[9] > 3 or
    #     qq[10] <  -3 or qq[10] > 3 or
    #     qq[11] <  -3 or qq[11] > 3 or
    #     qq[12] <  -3 or qq[12] > 3 
    #     ):

    if (qq[0] < -2 or qq[0] > 12 or
        qq[1] <  -1 or qq[1] > 5 or
        qq[2] <  -4 or qq[2] > 4 or
        qq[3] <  -4 or qq[3] > 4 or
        qq[4] <  -3 or qq[4] > 3 or
        qq[5] <  -3 or qq[5] > 3 or
        qq[6] <  -3 or qq[6] > 3 
        ):

        return True
    else:
        return False
    



if __name__ == '__main__':
    set_printoptions(precision=3)

    kk = Kalman6DOF()

    dt = 0.02

    Nt = 200000

    actions = array([[-1, 0],
                     [ 0,-1],
                     [ 0, 0],
                     [ 0, 1],
                     [ 1, 0],
                     ])

    initial_state = zeros(13)
    initial_state[6] = -1.0


    policy = { quantize(initial_state): [0,0] }

    states = [(0,initial_state)]

    next_states = []

    while states != []:
        k,ss = states.pop(0) ## pick next state in the queue
        if k > 50:
            print("too many iters")
            continue
        for aa in actions:
            
            kk.state = ss
            kk.predict_state_simulation(-dt, cal_acc_control_rocket, aa)
            
            sd = kk.state
            sdq = quantize(sd)

            print(70*'-')
            print(ss)
            print(quantize(ss))
            print(aa)
            print(sd)
            print(sdq)
            if sdq in policy.keys():
                print(' - exists')
                continue
            elif out_of_bounds(sdq):
                print(' - out of bounds')
                continue
            else:
                print(' - cool, action: ', aa)
                policy[sdq] = aa
                states.append((k+1,sd))





