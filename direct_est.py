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
    ## New indices
    ii = arrj[argmin([((a[:,:2]-b[cc,:2])**2).sum() for cc in arrj])]
    return b[ii,:].ravel()


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

        ## The system state: 3d position, 3d velocity, 3d orientation
        ## (quaternion) + 3d angular velocity.
        self.state = zeros(13)
        self.state[6] = 1.0  ## a "zero quaternion"
        self.previous_state = zeros(13)
        self.previous_state[6] = 1.0

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

        ## Variance of the acceleration. (Should be properly measured
        ## and given as a parameter in the initialization.)
        # self.Caccel = 0.1
        self.Caccel = 100.0

        ## Covariance matrix from measurements. (Also has to be better
        ## determined and given as a parameter at initialization.)
        self.Cobs = identity(9) * 2.0

        # self.Cobs = identity(9) * 0.1

        # self.Cobs[2,2] = 1e6
        # self.Cobs[5,5] = 1e6
        # self.Cobs[8,8] = 1e6

        self.time = 0.0



    def predict_state(self, dt):
        ## Set the transition matrix from the current orientation
        transition_matrix(self.Mtrans, self.state[6:10], dt)
        acceleration_matrix(self.Maccel, self.state[6:10], dt)

        ## Store current state
        self.previous_state[:] = self.state
        ## Calculate the new state using the transition matrix
        self.state = dot(self.Mtrans, self.state)
        ## Re-normalize the quaternion. (Controversy ensues...)
        self.state[6:10] = self.state[6:10] / norm(self.state[6:10])

        ########################################################################
        ## Update covariance of state estimate
        ##
        ## Calculate covariance to be added from an assumed uniform
        ## random acceleration.
        Ctrans = self.Caccel * dot(self.Maccel,self.Maccel.T)
        self.Cstate = dot(dot(self.Mtrans,self.Cstate), self.Mtrans.T) + Ctrans


    def predict_state_simulation(self, dt, v_accel_fun, *v_accel_params):
        ## This method uses an external procedure to calculate the
        ## acceleration on time, that is integrated. We use the
        ## "half-step" method, where first the position is predicted
        ## in half step in the future, an dthe acceleration is
        ## calculated for that position. Then this acceleration valus
        ## is used to predict the full-step.

        ## Set the transition matrix from the current orientation in
        ## order to calculate just the half-step prediction.
        transition_matrix(self.Mtrans, self.state[6:10], dt*0.5)

        ## Calculate the half-step state using the transition matrix
        ## and assuming inertial motion.
        half_step = dot(self.Mtrans, self.state)

        ## Calculate acceleration from external function.
        v_accel = v_accel_fun(half_step, *v_accel_params)

        ## Now calculate the full-step transition matrices.
        transition_matrix(self.Mtrans, self.state[6:10], dt)
        acceleration_matrix(self.Maccel, self.state[6:10], dt)

        ## Store current state
        self.previous_state[:] = self.state
        ## Calculate the new state using the transition matrix
        self.state = dot(self.Mtrans, self.state)
        ## Add acceleration contribution
        self.state += dot(self.Maccel, v_accel)

        ## Re-normalize the quaternion. (Controversy ensues...)
        self.state[6:10] = self.state[6:10] / norm(self.state[6:10])


        self.time += dt




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

    ## Read input data, the points extracted using kinect.
    xx = loadtxt(sys.argv[1])[:,:9]
    xx = xx[1100:1600]

    ## Transform from Kinect data to 3D using approximate camera model.
    xx[:, [0,3,6]] -= 320
    xx[:, [1,4,7]] -= 240
    ## Multiply x and y coordinated by mean distance divided by approximate focal distance.
    xx[:, [0,1,3,4,6,7]] *= 1157.0/580.0

    for n in range(1,xx.shape[0]):
       xx[n] = assoc(xx[n-1], xx[n])

    kalman = Kalman6DOF()
    kalman.state[6] = 1.0 ## "0" Quaternion

    kalman.pts = \
        array([[ 79.23584838,  58.55128818, -43.88465773],
               [-28.53663772, -99.4676606 ,  90.4078352 ],
               [-49.32580004,  38.82926386, -41.75416534]])

    kalman.pts = kalman.pts[arrj[5]]


    kalman.state[0:3] = mean(xx[0].reshape(-1,3))

    kalman.Cstate = 100.0 * identity(13) ## Initial state covariance

    xout = zeros((xx.shape[0], 13))
    zout = zeros((xx.shape[0], 12))

    eout = zeros(xx.shape[0])
    
    # mout = zeros((350,9))


    mm = kalman.pts - kalman.pts.mean(0)

    dt = .042
    for n in range(xx.shape[0]):
        ## Read measured points and subtract their mean.
        dd = xx[n].reshape(3,3)
        dd -= dd.mean(0)        

        
        hh = zeros((3,3))
        for k in range(3):
            hh += dot(dd[k:k+1,:].T, mm[k:k+1,:])
        hh = hh / 3
        uu,ss,vv = svd(hh)
        R = dot(uu,vv.T)

        print R
        print mm
        print dot(dd,R.T)
        print 




    print '--> mean observation prediction error level:', mean(log10(((xx-zout[:,:9])**2).sum(1)))

    ion()

    figure(1)
    plot(xx, 'b+')
    plot(zout[:,:9], 'r-')

    figure(2)
    plot(eout)


