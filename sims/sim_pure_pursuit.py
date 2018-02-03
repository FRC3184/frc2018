import math
import matplotlib.pylab as plot
import time

from control.pursuit import PurePursuitController
from mathutils import Vector2
from control.pose import Pose


def radius_ratio(R, D):
    return (R - D/2)/(R + D/2)


if __name__ == '__main__':
    travel_x = []
    travel_y = []
    target_x = []
    target_y = []
    targetp_x = []
    targetp_y = []
    distance = []
    times = []
    width = 29.25 / 12
    max_speed = 13
    acc = 10
    hopper_x = -2.47 + width / 2 - width
    hopper_y = 6 + 6.5/12 + width / 2
    path = [Vector2(0, 0), Vector2(5, 0), Vector2(0, 5)]
    pose = Pose(0, 0, 0 * math.pi/4)
    speed = max_speed

    lookahead = 1
    dt = 1/1000
    current_time = 0
    lines = []

    pursuit = PurePursuitController(pose, path, lookahead)
    lines = pursuit.path.path

    start = time.perf_counter()
    while not pursuit.is_at_end(pose):
        current_time += dt
        if current_time >= 10:
            break
        try:
            curve = pursuit.curvature(pose, speed)
        except ValueError:
            print("Break")
            break
        if curve == 0:
            left_speed = right_speed = speed
        else:
            radius = 1 / curve
            if radius > 0:
                left_speed = speed
                right_speed = speed * radius_ratio(radius, width)
            else:
                right_speed = speed
                left_speed = speed * radius_ratio(-radius, width)

        left_dist = left_speed * dt
        right_dist = right_speed * dt
        vel = (left_speed + right_speed) / 2
        angular_rate = (right_speed - left_speed) / width

        pose.x += dt * speed * math.cos(pose.heading)
        pose.y += dt * speed * math.sin(pose.heading)
        pose.heading += dt * angular_rate
        # print("{}\t{}\t{}\t{}\t{}\t{}".format(time, pose.x, pose.y, target.x, target.y, pose.distance(path[-1])))

        travel_x += [pose.x]
        travel_y += [pose.y]
        distance += [pose.distance(path[-1])]
        times += [current_time]
    elapsed_realtime = time.perf_counter() - start
    print("Simulated {} seconds in {} real seconds".format(current_time, elapsed_realtime))
    plot.figure(2)
    for line in lines:
        line.plot(plot)
    plot.plot(travel_x, travel_y)
    # plot.plot(target_x, target_y)

    for wp in path:
        plot.plot(wp.x, wp.y, 'bo')
    plot.show()


