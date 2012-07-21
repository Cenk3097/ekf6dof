import numpy as np
cimport numpy as np
DTYPE = np.float32
ctypedef np.float32_t DTYPE_t
DTYPE2 = np.float64
ctypedef np.float64_t DTYPE2_t








cdef linear_motion(double* ous, double* ins, double dt):
    cdef int k
    ## Copy state
    for k in range(13):
        ous[k] = ins[k]
    ## Add velocity times elapsed time, position and orientation.
    ous[0] += dt * ins[3]
    ous[1] += dt * ins[4]
    ous[2] += dt * ins[5]
    ous[6] += dt * (-ins[7] * ins[10] - ins[8] * ins[11] - ins[9] * ins[12])
    ous[7] += dt * ( ins[6] * ins[10] - ins[9] * ins[11] + ins[8] * ins[12])
    ous[8] += dt * ( ins[9] * ins[10] + ins[6] * ins[11] - ins[7] * ins[12])
    ous[9] += dt * (-ins[8] * ins[10] + ins[7] * ins[11] + ins[6] * ins[12])

cdef acceleration_increment(double* ous, double* ins, double dt, double* accel):
    cdef double dt2_2 = 0.5*dt*dt
    ous[0] += dt2_2 * accel[0]
    ous[1] += dt2_2 * accel[1]
    ous[2] += dt2_2 * accel[2]
    ous[3] += dt * accel[0]
    ous[4] += dt * accel[1]
    ous[5] += dt * accel[2]
    ous[6] += dt2_2 * (-ins[7] * accel[3] - ins[8] * accel[4] - ins[9] * accel[5])
    ous[7] += dt2_2 * ( ins[6] * accel[3] - ins[9] * accel[4] + ins[8] * accel[5])
    ous[8] += dt2_2 * ( ins[9] * accel[3] + ins[6] * accel[4] - ins[7] * accel[5])
    ous[9] += dt2_2 * (-ins[8] * accel[3] + ins[7] * accel[4] + ins[6] * accel[5])
    ous[10] += dt * accel[3]
    ous[11] += dt * accel[4]
    ous[12] += dt * accel[5]

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
        normalize_orientation(ous)

        ## Calculate acceleration from external function.
        cdef np.ndarray accel = np.zeros([6], dtype=DTYPE2)
        accel[:] = v_accel_fun(out_state, *v_accel_params)
        cdef double * aa = <double*>accel.data

        linear_motion(ous, ins, dt)
        acceleration_increment(ous, ins, dt, aa)
        normalize_orientation(ous)





cdef extern double sqrt(double)

