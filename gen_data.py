from pylab import *
from quaternion import Quat

Nt = 80
tt = mgrid[0.0:Nt]
dt = 10.0

pp = array([
    [1.0, 0, 0],
    [-.6,.1,.3],
    [-.4,.2,-.1],
    [0,0,0]
    ])


xx = empty([Nt, 12])

tinyrot = Quat(4*array([0.001,0.01,-0.0002]))
q = Quat(0,0,0.0)

for n in xrange(Nt):
    t = tt[n] * dt
    q = tinyrot * q

    uu = array([.1*sin(0.6e-3 * 2*pi*t),
                .01 * sin(1.2e-3 * 2*pi*t),
                .12*cos(0.1 + 0.6e-3 * 2*pi*t)])

    ww = 0.1 * random(12)
    
    xx[n] = (dot(pp, q.rot())+uu).ravel() + ww



    print t, q

ion()

subplot(2,2,3)
for c in [0,3,6,9]:
    plot(xx[0,0+c], xx[0,2+c],'ro')
    plot(xx[:,0+c], xx[:,2+c], 'b-+')
axis('equal')

subplot(2,2,1)
for c in [0,3,6,9]:
    plot(xx[0,0+c], xx[0,1+c],'ro')
    plot(xx[:,0+c], xx[:,1+c], 'b-+')
axis('equal')

subplot(2,2,4)
for c in [0,3,6,9]:
    plot(xx[0,2+c], xx[0,1+c],'ro')
    plot(xx[:,2+c], xx[:,1+c], 'b-+')
axis('equal')
