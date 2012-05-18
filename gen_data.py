from pylab import *
from quaternion import Quat



def transition(q, dt):
    M = identity(7)
    M[:4,4:] = dt * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])
    return M
    
def accel(q, dt):
    M = zeros([7, 3])
    M[:4,:] = dt**2/2 * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])
    M[4:,:] = dt * identity(3)
    return M

Nt = 1000
dt = 0.5

pp = array([
    [1.0, 0, 0],
    [-.6,.1,.3],
    [-.4,.2,-.1],
    [0,0,0]
    ])


xx = empty([Nt, 12])
qq = empty([Nt, 7])

q = array([1.0,0,0,0,0,0,0])

for n in xrange(Nt):
    w_ = 0.01 * (rand(3)-0.5)
    F = transition(q, dt)
    G = accel(q, dt)
    q = dot(F, q) + dot(G, w_)

    q[:4] = q[:4]/norm(q[:4])

    xx[n] = (dot(pp, Quat(q[:4]).rot())).ravel()
    qq[n] = q

    print n, q


savetxt('sim_data.txt', xx)
savetxt('params.txt', qq)
