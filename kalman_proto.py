import sys
from pylab import *


def assoc(aa, bb):
    a = aa.reshape(-1,3)
    b = bb.reshape(-1,3)
    return b[arrj[argmin([((a[:,:2]-b[cc,:2])**2).sum() for cc in arrj])],:].ravel()


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

        ## Point coordinates in the body reference frame
        self.pts = array([
            [1.0, 0, 0],
            [-.6,.1,.3],
            [-.4,.2,-.1]
            ])

        ## The system state, 3d position, 3d orientation (quaternion)
        ## + 3d velocity and angular velocity.
        self.state = zeros(13)
        self.Cstate = zeros((13,13)) ## State covariance matrix

        ## Vector to store the predicted observation values, calculated for a given state.
        self.y_hat = zeros(9)

        ## The transition function jacobian matrix, F, and the
        ## obesrvation matrix H calculated at any given state. The
        ## transition depends on all the state variables, the
        ## observation depends only in the position and orientation,
        ## not the velocities.
        self.Mtrans = identity(13)
        self.Mobser = zeros([9,7])

        ## The transition jacobian regarding the acceleration. Used to
        ## compute the transition covariance.
        self.Maccel = zeros([13, 6])


    def make_transition(self, dt):
        ## Set the transition matrix from the current orientation
        transition_matrix(self.Mtrans, self.state[6:10], dt)

        ## Calculate the new state using the transition matrix
        self.state = dot(self.Mtrans, self.state)
        ## Re-normalize the quaternion. Controversy ensues...
        self.state[6:10] = self.state[6:10] / norm(self.state[6:10])

        ########################################################################
        ## Update covariance of state estimate
        ##
        ## Calculate covariance fm assumed uniform random acceleration.
        Q = 
        
        self.Cstate = dot(dot(self.Mtrans,self.Cstate), self.Mtrans.T) + Q


    def observation_prediction(self):
        ## Current estimated position and orientation
        x = self.state[0:3]
        q = self.state[6:10]
        ## Get rotation matrix from quaternion
        R = matrix_from_quaternion(q)

        ## Predicted observation values
        self.y_hat[0:3] = self.x + dot(self.pts[0], R)
        self.y_hat[3:6] = self.x + dot(self.pts[1], R)
        self.y_hat[6:9] = self.x + dot(self.pts[2], R)
        
        ## Assemble observation jacobian matrix, H
        ## Derivatives on centroid position
        self.Mobser[[0,1,2,3,4,5,6,7,8],\
                    [0,1,2,0,1,2,0,1,2]] = 1
        ## Set the derivatives on quaternion params
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
        self.Mobser[0:3,:] = dot(self.pts,drdq.T).reshape(9,4)




if __name__ == '__main__':

    xx = loadtxt(sys.argv[1])

    for n in range(1,xx.shape[0]):
        xx[n] = assoc(xx[n-1], xx[n])








