import numpy as np
cimport numpy as np
DTYPE = np.float32
ctypedef np.float32_t DTYPE_t
DTYPE2 = np.float64
ctypedef np.float64_t DTYPE2_t








cdef linear_motion(double* ous, double* ins, double dt):
    cdef int k
    ## Copy velocities
    for k in range(3,6):
        ous[k] = ins[k]
    for k in range(10,13):
        ous[k] = ins[k]

    cdef double qt[4]

    ## Calcualte the quaternion of the rotatio by the given time. Each
    ## parameters is calculated like a linear displacement.
    qt[1] = dt * ins[10]
    qt[2] = dt * ins[11]
    qt[3] = dt * ins[12]
    ## The first coefficient from the rotation quaternion.
    qt[0] = sqrt(1 - (qt[1] * qt[1] + qt[2] * qt[2] + qt[3] * qt[3]))

    ## The current position and prientation, plus velocity-times-elapsed-time
    ous[0] = ins[0] + dt * ins[3]
    ous[1] = ins[1] + dt * ins[4]
    ous[2] = ins[2] + dt * ins[5]
    ## http://en.wikipedia.org/wiki/Quaternion#Hamilton_product
    ## Calculate new orientation using the "total quaternion".
    # ous[6] = qa * ins[6] + dt * (-ins[10] * ins[7] - ins[11] * ins[8] - ins[12] * ins[9])
    # ous[7] = qa * ins[7] + dt * ( ins[10] * ins[6] + ins[11] * ins[9] - ins[12] * ins[8])
    # ous[8] = qa * ins[8] + dt * (-ins[10] * ins[9] + ins[11] * ins[6] + ins[12] * ins[7])
    # ous[9] = qa * ins[9] + dt * ( ins[10] * ins[8] - ins[11] * ins[7] + ins[12] * ins[6])
    ous[6] = qt[0] * ins[6] + -qt[1] * ins[7] - qt[2] * ins[8] - qt[3] * ins[9]
    ous[7] = qt[0] * ins[7] +  qt[1] * ins[6] + qt[2] * ins[9] - qt[3] * ins[8]
    ous[8] = qt[0] * ins[8] + -qt[1] * ins[9] + qt[2] * ins[6] + qt[3] * ins[7]
    ous[9] = qt[0] * ins[9] +  qt[1] * ins[8] - qt[2] * ins[7] + qt[3] * ins[6]

cdef accelerated_motion(double* ous, double* ins, double dt, double* accel):
    cdef double dt2_2 = 0.5*dt*dt

    ## Calculate translation.
    ous[0] = ins[0] + dt*ins[3] + dt2_2 * accel[0]
    ous[1] = ins[1] + dt*ins[4] + dt2_2 * accel[1]
    ous[2] = ins[2] + dt*ins[5] + dt2_2 * accel[2]

    ## Calculate velocity increments.
    ous[3] = ins[3] + dt * accel[0]
    ous[4] = ins[4] + dt * accel[1]
    ous[5] = ins[5] + dt * accel[2]
    ous[10] = ins[10] + dt * accel[3]
    ous[11] = ins[11] + dt * accel[4]
    ous[12] = ins[12] + dt * accel[5]

    cdef double qt[4]

    ## Calcualte the "total quaternion". Each parameters is calculated
    ## like a linear displacement.
    qt[1] = dt * ins[10] + dt2_2 * accel[3]
    qt[2] = dt * ins[11] + dt2_2 * accel[4]
    qt[3] = dt * ins[12] + dt2_2 * accel[5]
    ## The first coefficient from the rotation quaternion.
    qt[0] = sqrt(1 - (qt[1] * qt[1] + qt[2] * qt[2] + qt[3] * qt[3]))

    ## Calculate new orientation using the "total quaternion".
    ous[6] = qt[0] * ins[6] + -qt[1] * ins[7] - qt[2] * ins[8] - qt[3] * ins[9]
    ous[7] = qt[0] * ins[7] +  qt[1] * ins[6] + qt[2] * ins[9] - qt[3] * ins[8]
    ous[8] = qt[0] * ins[8] + -qt[1] * ins[9] + qt[2] * ins[6] + qt[3] * ins[7]
    ous[9] = qt[0] * ins[9] +  qt[1] * ins[8] - qt[2] * ins[7] + qt[3] * ins[6]


cdef normalize_orientation(double* ous):
    cdef double n = 0.0
    n += ous[6]*ous[6]
    n += ous[7]*ous[7]
    n += ous[8]*ous[8]
    n += ous[9]*ous[9]
    n = 1.0/sqrt(n)
    ous[6] *= n
    ous[7] *= n
    ous[8] *= n
    ous[9] *= n


class Simulator:

    def __init__(self):
        return

    def simulation_step(self,
                        np.ndarray[DTYPE2_t, ndim=1, mode="c"] out_state not None,
                        np.ndarray[DTYPE2_t, ndim=1, mode="c"] in_state not None,
                        double dt,
                        v_accel_fun, *v_accel_params):

        cdef double * ous = <double*>out_state.data
        cdef double * ins = <double*>in_state.data

        ## Calculate half-step by linear motion.
        linear_motion(ous, ins, 0.5*dt)

        ## Calculate acceleration from external function.
        cdef np.ndarray accel = np.zeros([6], dtype=DTYPE2)
        cdef double * aa = <double*>accel.data
        accel[:] = v_accel_fun(out_state, *v_accel_params)[:]

        accelerated_motion(ous, ins, dt, aa)





cdef extern double sqrt(double)

