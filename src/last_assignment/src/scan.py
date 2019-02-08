#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int16, String
from last_assignment.msg import Coordinate
import utils


class ScanObstacles:
    def __init__(self):
        self.inner_track = utils.calculate_shape(26)
        self.outer_track = utils.calculate_shape(29)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1)
        self.min_distance = 100
        self.min_dist_obst_to_lane = 1.5
        self.dist_car_to_obs = 65
        self.lane_pub = rospy.Publisher("/lane_switcher", String, queue_size=1)

    def callback(self, scan):

        # Positions in [m]
        car_position = rospy.wait_for_message("/expected_coordinates", Coordinate)

        # Get car position in m and convert it to cm
        car_position_xy = np.array((car_position.current_y * 100, car_position.current_x * 100))

        dist_to_obst_inner = 100
        dist_to_obst_outer = 100

        angles = []
        ranges = list(scan.ranges)
        ranges.reverse()
        num_ranges = len(ranges)
        for i in range(num_ranges):
            if ranges[i] <= 1.5 and ((i >= 0 and i <= 45) or (i >= 315 and i <= 360)):
                angles.append((np.deg2rad(i), ranges[i]))

        for ran in angles:
            # Add 20cm to x axle which is the distance between the barcode and the lidar sensor
            object_x = ran[1] * np.sin(ran[0]) + 0.1
            object_y = ran[1] * np.cos(ran[0])

            # The results are in [m]
            obj_world_coords = utils.get_world_coordinates(
                car_position.current_angle,
                car_position.current_y,
                car_position.current_x,
                object_x,
                object_y
            )[:2]

            obj_world_coords = obj_world_coords / 0.01

            # Get obstacle's closest point and distance to the inner lane. The results are in [cm]
            closest_point_inner, distance_point_inner = utils.get_closest_point(
                obj_world_coords[0],
                obj_world_coords[1],
                self.inner_track
            )

            # Get obstacle's closest point and distance to the outer lane. The results are in [cm]
            closest_point_outer, distance_point_outer = utils.get_closest_point(
                obj_world_coords[0],
                obj_world_coords[1],
                self.outer_track
            )

            print("scan: car_position_xy: ", car_position_xy)
            print("scan: object_world_coords: ", obj_world_coords)
            print("scan: closest_point_inner: ", closest_point_inner)
            print("scan: closest_point_outer: ", closest_point_outer)
            print("scan: distance_point_inner: ", distance_point_inner)
            print("scan: distance_point_outer: ", distance_point_outer)

            # Each point which is closer than 0.15 m to the center of your lane shall be considered "on lane"
            if distance_point_inner < self.min_dist_obst_to_lane:
                dist_to_obst_inner = np.linalg.norm(np.array(car_position_xy) - np.array(closest_point_inner)) - 12
                print("scan: dist_to_obst_inner: ", dist_to_obst_inner)
            if distance_point_outer < self.min_dist_obst_to_lane:
                dist_to_obst_outer = np.linalg.norm(np.array(car_position_xy) - np.array(closest_point_outer)) - 12
                print("scan: dist_to_obst_outer: ", dist_to_obst_outer)

        # If there is an obstacle on your lane within a certain distance, e.g. 0.5 to 1 m
        # (in front of your vehicle, w.r.t. the lane on which you want to drive), switch lanes -
        # but do not switch lanes if there is an obstacle on both lanes - then your car should stop.

        print("scan_results: ", car_position.distance_to_inner, car_position.distance_to_outer)
        if dist_to_obst_inner <= self.dist_car_to_obs and dist_to_obst_outer <= self.dist_car_to_obs:
            self.lane_pub.publish("stop")
            print("scan_result: Obstacles on the track. Car had to stop")
        elif dist_to_obst_inner <= self.dist_car_to_obs:
            self.lane_pub.publish("outer")
            print("scan_result: Change to outer lane")
        elif dist_to_obst_outer <= self.dist_car_to_obs:
            self.lane_pub.publish("inner")
            print("scan_result: change to inner lane")
        elif car_position.distance_to_inner < car_position.distance_to_outer:
            self.lane_pub.publish("inner")
            print("scan_result: change to inner lane")
        else:
            self.lane_pub.publish("outer")
            print("scan_result: change to inner lane")


def main():
    print("ScanObstacles Node launched")
    scan_object = ScanObstacles()
    rospy.init_node('scan_object', anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + ": started!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()





