from pylab import *

def transition_matrix(M, q, dt):
    M[0:3,3:6] = dt * identity(3)
    M[6:10,10:] = dt * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])

def acceleration_matrix(M, q, dt):
    M[0:3,0:3] = .5*dt**2 * identity(3)
    M[3:6,0:3] = dt * identity(3)

    M[6:10,3:] = .5*dt**2 * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])
    M[10:,3:] = dt * identity(3)

def matrix_from_quaternion(q):
    a,b,c,d = q
    return np.array([ [(a*a+b*b-c*c-d*d), (2*b*c-2*a*d),     (2*b*d+2*a*c)     ],
                      [(2*b*c+2*a*d),     (a*a-b*b+c*c-d*d), (2*c*d-2*a*b)     ],
                      [(2*b*d-2*a*c),     (2*c*d+2*a*b),     (a*a-b*b-c*c+d*d)] ] )



Nt = 1000
dt = 0.045

# pts = array([
#     [1.0, 0, 0],
#     [-.6,.1,.3],
#     [-.4,.2,-.1],
#     [0,0,0]
#     ])

pts = array([
    [1.0,0,0],
    [0,1.0,0],
    [-.5,-.5,0.0]
    ])

xx = empty([Nt, pts.ravel().shape[0]])
qq = empty([Nt, 13])

q = zeros(13)
q[6] = 1.0


F = identity(13)
G = zeros((13,6))

for n in xrange(Nt):
    w_ = 0.4 * (rand(6)-0.5)
    #w_ = array([0,0,0,0,0,1.0])/min(n+1,10)
    #w_ = 10.0*array([rand()-0.5,rand()-0.5,rand()-0.5,0,0,0])
    #w_ = 0.4*array([0,0,0,rand()-0.5,rand()-0.5,rand()-0.5])
    #w_ = 0.4*array([0,0,0,rand()-0.5,rand()-0.5,rand()-0.5])

    transition_matrix(F, q[6:10], dt)
    acceleration_matrix(G, q[6:10], dt)
    q = dot(F, q) + dot(G, w_)
    q[6:10] = q[6:10]/norm(q[6:10])

    R = matrix_from_quaternion(q[6:10])
    xx[n] = (q[0:3] + dot(pts, R.T)).ravel()
    qq[n] = q

    print n, q


savetxt('sim_data.txt', xx)
savetxt('params.txt', qq)
