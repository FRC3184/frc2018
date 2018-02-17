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
    width = 24 / 12
    max_speed = 16
    cruise_speed = 0.5
    acc = 0.1

    accel_dist = (1 / 2) * cruise_speed ** 2 / acc
    path = [Vector2(0, 0), Vector2(20, 0), Vector2(20, 14), Vector2(20.5, 14)]
    pose = Pose(2, 0, 0 * math.pi/4)

    _begin_pose = Vector2(pose.x, pose.y)
    _end_pose = path[-1]

    lookahead = 5
    dt = 1/1000
    current_time = 0
    lines = []

    pursuit = PurePursuitController(path, lookahead)
    lines = pursuit.path.path

    start = time.perf_counter()
    while not pursuit.is_at_end(pose):
        poz = pose
        dist_to_end = _end_pose.distance(poz)
        dist_to_begin = _begin_pose.distance(poz)

        if pursuit.is_approaching_end(poz) and dist_to_end < accel_dist:
            speed = cruise_speed * dist_to_end / accel_dist
        elif dist_to_begin < accel_dist:
            speed = cruise_speed * dist_to_begin / accel_dist
        else:
            speed = cruise_speed

        if speed < 0.1:
            speed = 0.1

        current_time += dt
        if current_time >= 10:
            break
        try:
            curve, cte = pursuit.curvature(pose, speed)
        except ValueError:
            print("Break")
            break
        speed *= max_speed

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


