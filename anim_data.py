from pylab import *



xx = loadtxt('sim_data.txt')
Nt = xx.shape[0]


figure(1, figsize=(8,4.5))
for n in range(Nt):
    x = xx[n]
    pp = x.reshape(-1,3)
    for p in pp:
        plot([0, p[0]], [0,p[2]], 'b-',lw=2)
        plot(p[0], p[2], 'ro')
    plot(0,0, 'ro')
    axis('equal')
    axis([-2,2,-2,2])
    savefig('%04d.png'%n)
    cla()
