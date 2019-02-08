#!/usr/bin/env python
import rospy
import utils
from nav_msgs.msg import Odometry
from last_assignment.msg import Coordinate
from tf.transformations import euler_from_quaternion


class CarPosition:
    def __init__(self):
        self.inner_track = utils.calculate_shape(26)
        self.outer_track = utils.calculate_shape(29)
        self.sub_odom = rospy.Subscriber("/localization/odom/3", Odometry, self.odom_callback, queue_size=1)
        self.coordinates_pub = rospy.Publisher("/expected_coordinates", Coordinate, queue_size=1)

        self.distance = 50
        f = open('/home/adripinto/catkin_ws_user/src/assignment9/src/final_coords.txt', 'r+')
        f.truncate(0)
        f.write('current_x, current_y, closest_x, closest_y, expected_x, expected_y, distance\n')
        f.close()

    def odom_callback(self, data):
        current_x = int(round(float(data.pose.pose.position.x) / 0.01))
        current_y = int(round(float(data.pose.pose.position.y) / 0.01))
        current_z = data.pose.pose.orientation.z
        current_w = float(data.pose.pose.orientation.w)

        _, _, current_angle = euler_from_quaternion((0, 0, current_z, current_w)) # yaw angle


        closest_inner, distance_inner = utils.get_closest_point(
            current_y,
            current_x,
            self.inner_track
        )

        expected_point_inner = utils.get_point_to_distance(
            self.inner_track,
            (current_y, current_x),
            closest_inner,
            self.distance
        )

        closest_outer, distance_outer = utils.get_closest_point(
            current_y,
            current_x,
            self.outer_track
        )

        expected_point_outer = utils.get_point_to_distance(
            self.outer_track,
            (current_y, current_x),
            closest_outer,
            self.distance
        )

        if distance_inner < distance_outer:
            lane = "inner"
        else:
            lane = "outer"

        #Logging the position sent by the camera (current_x, current_y) and the calculated position (calc_x, calc_y)
        #info = ("{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}\n".format(
        #    current_y,
        #    current_x,
        #    closest_inner[0],
        #    closest_inner[1],
        #    expected_point_inner[0],
        #    expected_point_inner[1],
        #    distance_inner
        #))
        #with open('/home/adripinto/catkin_ws_user/src/assignment10/src/final_coords.txt', 'a') as out:
        #    out.write(info)

        expected_coordinates = Coordinate()
        expected_coordinates.current_x = current_x * 0.01
        expected_coordinates.current_y = current_y * 0.01
        expected_coordinates.inner_x = closest_inner[0] * 0.01
        expected_coordinates.inner_y = closest_inner[1] * 0.01
        expected_coordinates.inner_proj_x = expected_point_inner[0] * 0.01
        expected_coordinates.inner_proj_y = expected_point_inner[1] * 0.01
        expected_coordinates.outer_x = closest_outer[0] * 0.01
        expected_coordinates.outer_y = closest_outer[1] * 0.01
        expected_coordinates.outer_proj_x = expected_point_outer[0] * 0.01
        expected_coordinates.outer_proj_y = expected_point_outer[1] * 0.01
        expected_coordinates.distance_to_inner = distance_inner
        expected_coordinates.distance_to_outer = distance_outer
        expected_coordinates.current_angle = current_angle
        expected_coordinates.lane = lane

        print("car_position: ", distance_inner, distance_outer)

        # Values published in [m]
        self.coordinates_pub.publish(expected_coordinates)


def main():
    print("CarPosition Node launched")
    rospy.init_node('car_position', anonymous=True)
    car_position = CarPosition()
    rospy.loginfo(rospy.get_caller_id() + ": started!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
