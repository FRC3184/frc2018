from systems.elevator import Elevator
import matplotlib.pylab as plot
import time

if __name__ == '__main__':
    elev = Elevator(mock=True)

    print("Elevator hold torque w/ carriage: {} lb-in".format(elev.get_hold_torque(20)))
    print("Elevator hold torque w/ carriage + extent: {} lb-in".format(elev.get_hold_torque(50)))
    print("Elevator stall torque: {} lb-in".format(elev.get_stall_torque()))

    t_begin = time.time()
    talon_points, freq = elev.gen_profile(70, 0)
    print(f"Elapsed time: {time.time() - t_begin} s")

    times = []
    voltages = []
    positions = []
    for i, pt in enumerate(talon_points):
        times.append(i / freq)
        voltages.append(pt.velocity)
        positions.append(elev.native_to_inches(pt.position))
    print("Travel time: {} seconds".format(times[-1]))

    plot.figure(1)
    plot.plot(times, voltages)
    plot.figure(2)
    plot.plot(times, positions)

    plot.show()