from pylab import *



xx = loadtxt('sim_data.txt')

ion()

subplot(2,2,3)
for c in [0,3,6,9]:
    plot(xx[-1,0+c], xx[-1,2+c],'ro')
    plot(xx[:,0+c], xx[:,2+c], 'b-')
axis('equal')

subplot(2,2,1)
for c in [0,3,6,9]:
    plot(xx[-1,0+c], xx[-1,1+c],'ro')
    plot(xx[:,0+c], xx[:,1+c], 'b-')
axis('equal')

subplot(2,2,4)
for c in [0,3,6,9]:
    plot(xx[-1,2+c], xx[-1,1+c],'ro')
    plot(xx[:,2+c], xx[:,1+c], 'b-')
axis('equal')



