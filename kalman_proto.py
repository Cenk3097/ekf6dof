import sys
from pylab import *

arrj = array([[0,1,2],
              [0,2,1],
              [1,0,2],
              [1,2,0],
              [2,0,1],
              [2,1,0]])
def assoc(aa, bb):
    a = aa.reshape(-1,3)
    b = bb.reshape(-1,3)
    return b[arrj[argmin([((a[:,:2]-b[cc,:2])**2).sum() for cc in arrj])],:].ravel()


def matrix_from_quaternion(q):
    a,b,c,d = q
    return np.array([ [(a*a+b*b-c*c-d*d), (2*b*c-2*a*d),     (2*b*d+2*a*c)     ],
                      [(2*b*c+2*a*d),     (a*a-b*b+c*c-d*d), (2*c*d-2*a*b)     ],
                      [(2*b*d-2*a*c),     (2*c*d+2*a*b),     (a*a-b*b-c*c+d*d)] ] )


def transition_matrix(M, q, dt):
    M[0:3,3:6] = dt * identity(3)
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


class Kalman6DOF:
    def __init__(self):

        ## Point coordinates in the body reference frame
        # self.pts = array([
        #     [1.0, 0, 0],
        #     [-.6,.1,.3],
        #     [-.4,.2,-.1]])

        self.pts = array([[-50.37333333,  66.99333333,  20.6       ],
                          [ 69.62666667, -17.00666667, -24.4       ],
                          [-11.37333333, -40.00666667,  -2.4       ]])



        ## The system state, 3d position, 3d orientation (quaternion)
        ## + 3d velocity and angular velocity.
        self.state = zeros(13)
        self.Cstate = zeros((13,13)) ## State covariance matrix

        ## Vector to store the predicted observation values, calculated for a given state.
        self.z_hat = zeros(9)

        ## The transition function jacobian matrix, F, and the
        ## obesrvation matrix H calculated at any given state. The
        ## transition depends on all the state variables, the
        ## observation depends only in the position and orientation,
        ## not the velocities.
        self.Mtrans = identity(13)
        self.Mobser = zeros([9,13])

        ## The transition jacobian regarding the acceleration. Used to
        ## compute the transition covariance.
        self.Maccel = zeros([13, 6])

        ## Variance of the acceleration. (Must be properly measured
        ## and given as a parameter in the initialization.)
        self.accel_var = 1.0

        ## Covariance matrix from measurements. (Also has to be better
        ## determined and given as a parameter at initialization.)
        self.R = identity(9) * 3.0

    def predict_state(self, dt):
        ## Set the transition matrix from the current orientation
        transition_matrix(self.Mtrans, self.state[6:10], dt)
        acceleration_matrix(self.Maccel, self.state[6:10], dt)

        ## Calculate the new state using the transition matrix
        self.state = dot(self.Mtrans, self.state)
        ## Re-normalize the quaternion. Controversy ensues...
        self.state[6:10] = self.state[6:10] / norm(self.state[6:10])

        ########################################################################
        ## Update covariance of state estimate
        ##
        ## Calculate covariance to be added from an assumed uniform
        ## random acceleration.
        QQt = self.accel_var * dot(self.Maccel,self.Maccel.T)
        self.Cstate = dot(dot(self.Mtrans,self.Cstate), self.Mtrans.T) + QQt


    def predict_observations(self):
        ## Current estimated position and orientation
        x = self.state[0:3]
        q = self.state[6:10]
        ## Get rotation matrix from quaternion
        R = matrix_from_quaternion(q)

        ## Predicted observation values
        self.z_hat[0:3] = self.state[0:3] + dot(self.pts[0], R)
        self.z_hat[3:6] = self.state[0:3] + dot(self.pts[1], R)
        self.z_hat[6:9] = self.state[0:3] + dot(self.pts[2], R)
        
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
                        [ b, c, d]])
        self.Mobser[:,6:10] = dot(self.pts, drdq.T).reshape(9,4)

    def update_from_observations(self, z):
        ## Calculate residue from the emasured and predicted observations.
        residue = z - self.z_hat
        Cresidue = dot(dot(self.Mobser, self.Mtrans), self.Mobser.T) + self.R

        ## Kalman gain
        self.K = dot(dot(self.Mtrans, self.Mobser.T), inv(Cresidue))

        ## Incorporate new observations into state estimation.
        self.state += dot(self.K, residue)
        self.Cstate += -dot(self.K, self.Mobser)

        ## Re-normalize the quaternion. Controversy ensues again.
        self.state[6:10] = self.state[6:10] / norm(self.state[6:10])




if __name__ == '__main__':

    xx = loadtxt(sys.argv[1])

    for n in range(1,xx.shape[0]):
        xx[n] = assoc(xx[n-1], xx[n])



    
    kalman = Kalman6DOF()


    kalman.state[0:3] = array([336.37333333,   227.00666667,  2801.4])
    kalman.state[3] = 0.1
    kalman.state[6] = 1.0
    #kalman.state[10] = 1.0
    kalman.Cstate = 10.0 * identity(13)

    out = zeros((xx.shape[0], 7))

    #dt = 0.042
    dt = .1
    for n in range(10):#range(xx.shape[0]):
        print 70*'-'
        kalman.predict_state(dt)
        print 'pred st: ', kalman.state
        kalman.predict_observations()
        print 'pred obs:', kalman.z_hat
        kalman.update_from_observations(xx[n])
        print 'measured:', xx[n]
        print 'updt st:', kalman.state

        out[n,0:3] = kalman.state[0:3]
        out[n,3:7] = kalman.state[6:10]
