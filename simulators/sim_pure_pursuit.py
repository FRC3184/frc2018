import math
import time

import matplotlib.pylab as plot

from control.pose import Pose
from control.pursuit import PurePursuitController
from mathutils import Vector2


def radius_ratio(R, D):
    return (R - D/2)/(R + D/2)


def simulate(path, lookahead, do_plot=False, do_print=False, print_danger=False, cruise_speed=0.6, acc=0.6,
             error_factor: float=1):
    travel_x = []
    travel_y = []
    distance = []
    times = []
    width = 24 / 12
    max_speed = 16

    accel_dist = (1 / 2) * (cruise_speed * max_speed) ** 2 / (acc * max_speed)

    pose = Pose(path[0].x, path[0].y, 0 * math.pi/4)

    _begin_pose = Vector2(pose.x, pose.y)
    _end_pose = path[-1]

    dt = 1/1000
    current_time = 0

    pursuit = PurePursuitController(path, lookahead)
    lines = pursuit.path.path

    start = time.perf_counter()
    score = 0
    while not pursuit.is_at_end(pose):
        poz = pose
        dist_to_end = _end_pose.distance(poz)
        dist_to_begin = _begin_pose.distance(poz)

        if pursuit.is_approaching_end(poz) and dist_to_end < accel_dist:
            speed = max_speed * cruise_speed * dist_to_end / accel_dist
        elif dist_to_begin < accel_dist:
            speed = max_speed * cruise_speed * dist_to_begin / accel_dist
        else:
            speed = max_speed * cruise_speed

        minspeed = 0.1 * max_speed
        if speed < minspeed:
            speed = minspeed

        # speed = max_speed

        current_time += dt
        if current_time >= 15:
            break

        curve, cte = pursuit.curvature(pose, speed / max_speed)

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

        pose.x += dt * vel * math.cos(pose.heading)
        pose.y += dt * vel * math.sin(pose.heading)
        pose.heading += dt * angular_rate
        # print("{}\t{}\t{}\t{}\t{}\t{}".format(time, pose.x, pose.y, target.x, target.y, pose.distance(path[-1])))

        travel_x += [pose.x]
        travel_y += [pose.y]
        distance += [pose.distance(path[-1])]
        times += [current_time]
    elapsed_realtime = time.perf_counter() - start
    score = score/(current_time/dt)
    cte = abs(pursuit.get_endcte(pose))
    if do_print:
        print("Simulated {} seconds in {} real seconds".format(current_time, elapsed_realtime))
        print(f"Final CTE: {cte}")
        print(f"Radius score: {score}")

    if do_plot:
        for line in lines:
            line.plot(plot)
        plot.plot(travel_x, travel_y)

        for wp in path:
            plot.plot(wp.x, wp.y, 'bo')
        # plot.show()

    return score, cte


if __name__ == '__main__':
    path = [Vector2(0, 0), Vector2(5, 0), Vector2(5, 5), Vector2(10, 10)]
    simulate(path, 2, do_plot=True, do_print=True)
    plot.show()