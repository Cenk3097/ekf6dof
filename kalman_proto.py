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
    M[6:10,10:13] = dt * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])

def acceleration_matrix(M, q, dt):
    M[0:3,0:3] = .5*dt**2 * identity(3)
    M[3:6,0:3] = dt * identity(3)
    M[6:10,3:6] = .5*dt**2 * array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
        ])
    M[10:13,3:] = dt * identity(3)

class Kalman6DOF:
    def __init__(self):

        ## Point coordinates in the body reference frame

        # self.pts = array([
        #     [1.0,0,0],
        #     [0,1.0,0],
        #     [-.5,-.5,0.0]
        #     ]) ## simulation
        # self.pts = array([[-50.37333333,  66.99333333,  20.6       ],
        #                   [ 69.62666667, -17.00666667, -24.4       ],
        #                   [-11.37333333, -40.00666667,  -2.4       ]]) ## tree3
        # self.pts = array([[14.74758755,  -9.02537984, -41.20558299 ],
        #                   [2.21633152, -3.92949907,  61.57078696   ],
        #                   [3.93343197,   0.71211179, -11.49110769  ]]) ## tree_22may
        # self.pts = identity(3)
        self.pts = array([
            [1.0,0,0],
            [0,1.0,0],
            [0,0.5,1.0]
            ])



        ## The system state, 3d position, 3d orientation (quaternion)
        ## + 3d velocity and angular velocity.
        self.state = zeros(13)
        self.previous_state = zeros(13)
        #self.Cstate = zeros((13,13)) ## State covariance matrix
        #self.Cstate = 0.1 * identity(13)

        ## Vector to store the predicted observation values,
        ## calculated for a given state. The values are the 9 point
        ## coordinates followed by the 6 velocoty parameters.
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
        self.Caccel = 0.1

        ## Covariance matrix from measurements. (Also has to be better
        ## determined and given as a parameter at initialization.)
        self.Cobs = identity(9) * 1.0
        # self.Cobs[2,2] = 1e6
        # self.Cobs[5,5] = 1e6
        # self.Cobs[8,8] = 1e6


    def predict_state(self, dt):
        ## Set the transition matrix from the current orientation
        transition_matrix(self.Mtrans, self.state[6:10], dt)
        acceleration_matrix(self.Maccel, self.state[6:10], dt)

        ## Store current state
        self.previous_state[:] = self.state
        ## Calculate the new state using the transition matrix
        self.state = dot(self.Mtrans, self.state)
        ## Re-normalize the quaternion. Controversy ensues...
        self.state[6:10] = self.state[6:10] / norm(self.state[6:10])

        ########################################################################
        ## Update covariance of state estimate
        ##
        ## Calculate covariance to be added from an assumed uniform
        ## random acceleration.
        Ctrans = self.Caccel * dot(self.Maccel,self.Maccel.T)
        self.Cstate = dot(dot(self.Mtrans,self.Cstate), self.Mtrans.T) + Ctrans


    def predict_observations(self):
        ## Current estimated position and orientation
        c = self.state[0:3]
        q = self.state[6:10]
        ## Get rotation matrix from quaternion
        R = matrix_from_quaternion(q)

        ## Predicted observation values
        self.z_hat[:] = (dot(self.pts, R.T) + c).ravel()
        
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
        self.Mobser[:9,6:10] = dot(self.pts, drdq.T).reshape(9,4)

    def update_from_observations(self, z):

        ## Calculate residue from the measured and predicted observations.
        residue = z - self.z_hat
        Cresidue = dot(dot(self.Mobser, self.Cstate), self.Mobser.T) + self.Cobs

        ## Kalman gain
        self.K = dot(dot(self.Cstate, self.Mobser.T), inv(Cresidue))

        ## Incorporate new observations into state estimation.
        self.state = self.state + dot(self.K, residue)
        self.Cstate = dot(identity(13) - dot(self.K, self.Mobser),self.Cstate)

        ## Re-normalize the quaternion. Controversy ensues again.
        self.state[6:10] = self.state[6:10] / norm(self.state[6:10])




if __name__ == '__main__':

    xx = loadtxt(sys.argv[1])[:,:9]

    # for n in range(1,xx.shape[0]):
    #     xx[n] = assoc(xx[n-1], xx[n])

    
    kalman = Kalman6DOF()

    kalman.state[6] = 1.0 ## "0" Quaternion

    # kalman.state[0:3] = array([0.0,0.0,0.0]) ## simulation
    # kalman.state[0:3] = array([336.37333333,   227.00666667,  2801.4]) ## Real data tree3
    # kalman.state[0:3] = array([  328.07273667,   220.70310667,  1154.43393333]) # tree_22may
    kalman.state[0:3] = mean(xx[0].reshape(-1,3))

    #kalman.state[13+7] = 0.5
    
    kalman.Cstate = 100.0 * identity(13) ## Initial state covariance


    xout = zeros((xx.shape[0], 13))
    zout = zeros((xx.shape[0], 12))


    dt = .042
    for n in range(xx.shape[0]):
        print 70*'-'
        kalman.predict_state(dt)
        print 'pred st: ', kalman.state
        kalman.predict_observations()
        print 'pred obs:', kalman.z_hat
        kalman.update_from_observations(xx[n])
        print 'measured:', xx[n]
        print 'updt st:', kalman.state

        ## Log predicted state and outputs
        xout[n] = kalman.state
        zout[n,:9] = kalman.z_hat
        zout[n,9:12] = kalman.state[:3]



    ion()
    plot(xx, 'b+')
    plot(zout[:,:9], 'r-')

    print '--> mean observation prediction error level:', mean(log10(((xx-zout[:,:9])**2).sum(1)))
