from pylab import *



xx = loadtxt(sys.argv[1])

Nt = xx.shape[0]


figure(1, figsize=(8,4.5))
for n in range(Nt):
    x = xx[n]
    pp = x.reshape(-1,3)
    for p in pp[:-1]:
        plot([pp[-1,0], p[0]], [pp[-1,1],p[1]], 'b-',lw=2)
        plot(p[0], p[1], 'ro')
    plot(pp[-1,0],pp[-1,1], 'ro')
    axis('equal')
    #axis([-2,2,-2,2])
    axis([100,540,70,420])

    savefig('frames/%08d.png'%n)
    cla()
