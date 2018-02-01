from control.MotionProfile import MotionProfile
import matplotlib.pylab as plot


if __name__ == '__main__':
    mp = MotionProfile(0, 10, 3, 1.5)
    times = []
    posz = []
    velz = []
    accs = []
    for pt in mp._points:
        times.append(pt.time)
        posz.append(pt.position)
        velz.append(pt.velocity)
        accs.append(pt.acc)

    plot.plot(times, posz)
    plot.plot(times, velz)
    plot.plot(times, accs)

    plot.show()