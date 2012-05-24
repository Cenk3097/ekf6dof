
from pylab import *
import scipy.optimize


import sys

xx = loadtxt(sys.argv[1])[:,:9]

def triangulate(m):
    # m = dot(h,h.T)
    ma = sqrt(m[0,0])
    mb = m[0,1]/ma
    md = m[0,2]/ma
    mc = sqrt(m[1,1]-mb**2)
    me = (m[1,2] - mb*md )/mc
    mf = m[2,2] - md**2 - me**2

    return array([[ma,0,0],
                  [mb,mc,0],
                  [md,me,mf]])
    

def func(xx, zz):
    x = xx.reshape(6,3)

    c = x[0]
    v = x[1]
    ww = x[2]
    H = x[3:6].reshape(3,3)
    
    w = array([[     1, ww[2],-ww[1]],
               [-ww[2],     1, ww[0]],
               [ ww[1],-ww[0],     1]])

    fun = zeros((3,3,3))
    fun[0] = dot(H, w.T) + c - v
    fun[1] =     H       + c
    fun[2] = dot(H,   w) + c + v

    return zz - fun.ravel()


def cross_matrix(v):
    return array([[0,v[2],-v[1]],[-v[2],0,v[0]],[v[1],-v[0],0]])

def jaco(xx, zz):
    out = zeros((27,18))

    x = xx.reshape(6,3)

    c = x[0]
    v = x[1]
    ww = x[2]
    H = x[3:6].reshape(3,3)
    
    w = array([[     1, ww[2],-ww[1]],
               [-ww[2],     1, ww[0]],
               [ ww[1],-ww[0],     1]])

    ## Derivatives in c
    for k in range(0,27,3):
        out[k:k+3,0:3] = identity(3)
    ## Derivatives in v
    for k in range(0,9,3):
        out[k:k+3,3:6] = -identity(3)
    for k in range(18,27,3):
        out[k:k+3,3:6] = identity(3)
    ## Derivatives in w
    for k in range(0,27,3):
        out[k:k+3,6:9] = cross_matrix(H[(k/3)%3])

    ## Derivatives in h
    for k in range(0,9,3):
        out[   k:   k+3,9+k:9+k+3] = w
        out[ 9+k: 9+k+3,9+k:9+k+3] = identity(3)
        out[18+k:18+k+3,9+k:9+k+3] = w.T

    return out


#xx = xx[::3]

mm = zeros((3,3))
Np = xx.shape[0]
for n in range(Np-3):
    zn = xx[n]
    xini = zeros(18)
    xini[:3] = zn.mean(0)
    xini[9:] = identity(3).ravel()
    zz = xx[n:n+3].ravel()
    
    #sol,lixo = scipy.optimize.leastsq(func, xini, args=(zz,), Dfun=jaco)
    sol,lixo = scipy.optimize.leastsq(func, xini, args=(zz,))
    #print sol.reshape(-1,3)

    h = sol[9:].reshape(3,3)
    mm = mm * .99 + 0.01 * dot(h,h.T)
    print mm
    print triangulate(mm)


    
