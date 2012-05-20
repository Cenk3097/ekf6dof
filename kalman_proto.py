import sys
from pylab import *


def matrix_from_quaternion(q):
    a,b,c,d = q
    return np.array([ [(a*a+b*b-c*c-d*d), (2*b*c-2*a*d),     (2*b*d+2*a*c)     ],
                      [(2*b*c+2*a*d),     (a*a-b*b+c*c-d*d), (2*c*d-2*a*b)     ],
                      [(2*b*d-2*a*c),     (2*c*d+2*a*b),     (a*a-b*b-c*c+d*d)] ] )

def matrix_from_quaternion_derivatives(q):




def transition_matrix(M, q, dt):
    M[6:10,10:] = dt * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])

def acceleration_matrix(M, q, dt):
    M[:3,:3] = dt**2/2 * identity(3)
    M[3:6,3:6] = dt * identity(3)
    M[6:10,3:] = dt**2/2 * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])
    M[10:,3:] = dt * identity(3)

def measurement_matrix(M, ):
    M[6:10,10:] = dt * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])


def class Kalman6DOF:
    def __init__(self):
        self.state = zeros(13)

        self.Mtrans = identity(13)
        self.Mobser = zeros([9,13])

        self.Maccel = zeros([13, 6])

        self.pts = array([
            [1.0, 0, 0],
            [-.6,.1,.3],
            [-.4,.2,-.1]
            ])

    def make_transition(self, dt):
        transition_matrix(self.Mtrans, self.state[6:10], dt)
        self.state = dot(self.Mtrans, self.state)

        ## Re-normalize the quaternion. Controversy ensues...
        self.state[6:10] = self.state[6:10] / norm(self.state[6:10])


    def observation_prediction(self):
        ## Get rotation matrix from quaternion
        q = state[6:10]
        matrix_from_quaternion(q)

        a,b,c,d = q
        drdq = 2*array([[ a,-d, c],
                        [ b, c, d],
                        [-c, b, a],
                        [-d,-a, b],
                        [ d, a,-b],
                        [ c,-b,-a],
                        [ b, c, d],
                        [ a,-d, c],
                        [-c, b, a],
                        [ d, a,-b],
                        [-a, d,-c],
                        [ b, c, d]])]
        
        ## Derivatives on centroid position
        self.Mobser[[0,1,2,3,4,5,6,7,8],\
                    [0,1,2,0,1,2,0,1,2]] = 1

        ## Derivatives on quaternion params
        self.Mobser[0:3,:] = dot(self.pts,drdq.T).reshape(9,4)


if __name__ == '__main__':

    xx = loadtxt(sys.argv[1])

    for n in range(1,xx.shape[0]):
        xx[n] = assoc(xx[n-1], xx[n])








