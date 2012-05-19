import sys
import matplotlib
matplotlib.use('Agg')

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



xx = loadtxt(sys.argv[1])

for n in range(1,xx.shape[0]):
    xx[n] = assoc(xx[n-1], xx[n])

ion()

dir_colors=['#ea6949', '#51c373', '#a370ff', '#444444']

#figure(1, figsize=(6.40,3.60),dpi=70)
figure(1, figsize=(12.80,7.20),dpi=70)

for LIM in mgrid[3:xx.shape[0]:6]:
    sl = mgrid[clip(LIM-150,0,50000):LIM]

    subplot(2,2,1)
    cla()
    for c in [0,3,6]:
        plot(xx[sl,0+c], xx[sl,1+c], '-',color=dir_colors[c/3])
        plot(xx[sl[-1],0+c], xx[sl[-1],1+c],'o',color=dir_colors[c/3],mew=2)
    axis('equal')
    axis([100,540,70,420])

    subplot(2,2,3)
    cla()
    for c in [0,3,6]:
        plot(xx[sl,0+c], xx[sl,2+c], '-',color=dir_colors[c/3])
        plot(xx[sl[-1],0+c], xx[sl[-1],2+c],'o',color=dir_colors[c/3],mew=2)
    axis('equal')
    axis([100,540,2700,2900])

    subplot(2,2,4)
    cla()
    for c in [0,3,6]:
        plot(xx[sl,1+c], xx[sl,2+c], '-',color=dir_colors[c/3])
        plot(xx[sl[-1],1+c], xx[sl[-1],2+c],'o',color=dir_colors[c/3],mew=2)
    axis('equal')
    axis([70,420,2700,2900])


    sl = mgrid[clip(LIM-300,0,50000):LIM]
    subplot(2,2,2)
    cla()
    plot([sl[-1],sl[-1]], [100,600], 'r--', lw=1)
    for c in range(3):
        plot(sl, xx[sl,3*c+0], '-',color=dir_colors[c])
        plot(sl, xx[sl,3*c+1], '-',color=dir_colors[c])
        plot(sl, xx[sl,3*c+2], '-',color=dir_colors[c])
        plot(sl[-1], xx[sl[-1],3*c+0], 'o',color=dir_colors[c],mew=2)
        plot(sl[-1], xx[sl[-1],3*c+1], 'o',color=dir_colors[c],mew=2)
        plot(sl[-1], xx[sl[-1],3*c+2], 'o',color=dir_colors[c],mew=2)
    axis([LIM-300, sl[-1]+50, 70, 540])

    savefig('frames/%08d.png'%LIM, dpi=100)

