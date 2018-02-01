from control.MotionProfile import MotionProfile
import matplotlib.pylab as plot


def plot_mp(mp):
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


if __name__ == '__main__':
    rev_profile = MotionProfile(10, -10, 3, 1.5)
    for_profile = MotionProfile(-10, 10, 3, 1.5)
    plot_mp(for_profile)
    plot_mp(rev_profile)
