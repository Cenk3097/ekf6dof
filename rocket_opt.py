

from __future__ import print_function
from pylab import *

from simple_kalman import *




def cal_acc_control_rocket(x, c, aa):
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

    k1 = aa[c,0]*10.0
    k2 = aa[c,1]*10.0

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

    kk = Kalman6DOF()

    dt = 0.1



    lowlim = array([-1,-1,-5,-5,-4,-4,-5], dtype=int16)
    higlim = array([10, 5, 5, 5, 4, 4, 5], dtype=int16)
    parmrg = higlim - lowlim + 1


    actions = array([[-1, 0],
                     [ 0,-1],
                     [ 0, 0],
                     [ 0, 1],
                     [ 1, 0],
                     ])

    initial_state = zeros(13)
    initial_state[6] = 1.0


    policy = { quantize(initial_state, lowlim, parmrg): [0,0] }

    states = [(0,initial_state)]

    next_states = []

    while states != []:
        k,ss = states.pop(0) ## pick next state in the queue
        if k > 200:
            print("too many iters")
            continue
        for aa in range(5):
            
            kk.state = ss
            kk.predict_state_simulation(-dt, cal_acc_control_rocket, aa, actions)
            
            sd = kk.state

            print(quantize(ss, lowlim, parmrg), aa, end=' ')

            try:
                sdq = quantize(sd, lowlim, parmrg)
            except ValueError:
                print(' - out')
                continue

            print(sdq, end='')
            if sdq in policy.keys():
                print(' - exists')
                continue
            else:
                print(' - record')
                policy[sdq] = aa
                states.append((k+1,sd))


    import pickle
    pickle.dump(policy, open('policy.p', 'wb'))



