#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys
# from tf.transformations import euler_from_quaternion

img = cv2.imread('/home/adripinto/catkin_ws_user/src/assignment9/src/map.jpeg')
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
black = np.zeros(gray.shape)

class Oval:
    def __init__(self, width, upper_semicircle, lower_semicircle, right_segment, left_segment):
        self.width = width
        self.upper_semicircle = upper_semicircle
        self.lower_semicircle = lower_semicircle
        self.left_segment = left_segment
        self.right_segment = right_segment

    def __str__(self):
        return str(
            "Width: %d, Upper Semicircle: %d, %d, %d, Lower Semicircle: %d, %d, %d, Left Segment: %d, Right Segment:, %d "
            % (
                self.width,
                self.upper_semicircle[0],
                self.upper_semicircle[1],
                self.upper_semicircle[2],
                self.lower_semicircle[0],
                self.lower_semicircle[1],
                self.lower_semicircle[2],
                self.left_segment,
                self.right_segment
            )
        )


def get_point_circle(radius, x, h, k):
    #(h,k) is the circle center
    return round(np.sqrt(radius**2 - (x - h)**2) + k)


def get_contours(img, contour_id):
    ret, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(thresh, 1, 2)
    return contours[contour_id] #26 inner oval, 29 outer oval


def distance_to_the_circle(x, y, h, k, r):
    # (h,k) is the center point of the circle and r is radius
    return abs(np.sqrt((x - h)**2 + (y - k)**2) - r)


def get_intercept_point(x, y, h, k, r): # Closest point to the semicircle
    # https://math.stackexchange.com/questions/127613/closest-point-on-circle-edge-from-point-outside-inside-the-circle
    vector_x = x - h
    vector_y = y - k
    magnitud_vector = np.sqrt(vector_x**2 + vector_y**2)
    point_x = round(h + (vector_x / magnitud_vector) * r)
    point_y = round(k + (vector_y / magnitud_vector) * r)
    return point_x, point_y


def get_point_to_distance(oval, point_1, point_2, d):
    # point_1 --> position of car
    # point_2 --> closest point on the oval
    D = np.sqrt((point_2[0] - point_1[0])**2 + (point_2[1] - point_1[1])**2)
    if point_1 == point_2:
        return point_1

    x = point_2[0] + (d / D) * (point_2[0] - point_1[0])
    y = point_2[1] + (d / D) * (point_2[1] - point_1[1])

    if oval.lower_semicircle[1] > point_1[1] > oval.upper_semicircle[1]:
        if point_1[0] <= oval.left_segment or point_1[0] >= oval.right_segment:
            x = point_2[0] - (d / D) * (point_2[0] - point_1[0])
            y = point_2[1] - (d / D) * (point_2[1] - point_1[1])
    elif point_1[1] <= oval.upper_semicircle[1]:
        dist = np.sqrt((point_1[0] - oval.upper_semicircle[0])**2 + (point_1[1] - oval.upper_semicircle[1])**2)
        if dist >= oval.upper_semicircle[2]:
            x = point_2[0] - (d / D) * (point_2[0] - point_1[0])
            y = point_2[1] - (d / D) * (point_2[1] - point_1[1])
    elif point_1[1] >= oval.lower_semicircle[1]:
        dist = np.sqrt((point_1[0] - oval.lower_semicircle[0]) ** 2 + (point_1[1] - oval.lower_semicircle[1]) ** 2)
        if dist >= oval.lower_semicircle[2]:
            x = point_2[0] - (d / D) * (point_2[0] - point_1[0])
            y = point_2[1] - (d / D) * (point_2[1] - point_1[1])

    return (int(round(x)), int(round(y)))



def find_main_parts(cnt):
    # Split contours into 2 half circles and two straight lines
    # coord[1] --> right line
    # coord[2] --> lower semicircle
    # coord[3] --> left line
    # coord[4] --> upper semicircle
    size_cnt = len(cnt)
    groups = 0
    coord = []
    coord.append([])
    for i in range(1, size_cnt):
        if abs(cnt[i][0][1] - cnt[i - 1][0][1]) > 100:
            groups += 1
            coord.append([])
            coord[groups].append(cnt[i - 1][0])
            coord[groups].append(cnt[i][0])
            groups += 1
            coord.append([])
        coord[groups].append(cnt[i][0])

    for i in range(len(coord)):
        coord[i] = np.array(coord[i])

    coord[4] = np.append(coord[4], coord[0], axis=0)
    return coord


def calculate_shape(contour_id):
    # Load the image and convert it to gray
    img = cv2.imread('/home/adripinto/catkin_ws_user/src/assignment9/src/map.jpeg')

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # Find contours
    cnt = get_contours(gray, contour_id)

    # Draw contours
    # black = np.zeros(gray.shape)
    cv2.drawContours(black, [cnt], -1, (255, 255, 30), 3)
    plt.imshow(black)

    coord = find_main_parts(cnt)

    # Fit ellipses and print center points and radius of the semicircles
    ellipse1 = cv2.fitEllipse(coord[4])
    x1 = ellipse1[0][0]
    y1 = ellipse1[0][1]
    r1 = round(ellipse1[1][0] / 2)
    print("Ellipse1 Center:", (int(round(x1)), int(round(y1))), "Radius:", int(round(r1)))

    ellipse2 = cv2.fitEllipse(coord[2])
    x2 = ellipse2[0][0]
    y2 = ellipse2[0][1]
    r2 = round(ellipse2[1][0] / 2)
    print("Ellipse2 Center:", (int(round(x2)), int(round(y2))), "Radius:", int(round(r2)))

    # Print semicircle equations
    print("Upper semi-circle equation: sqrt(", r1, "**2 - (x -", round(x1), ")**2 + ", round(y1))
    print("Lower semi-circle equation: sqrt(", r2, "**2 - (x -", round(x2), ")**2 + ", round(y2))

    # Print line equations
    print("Equation right-line:", 0, "x +", coord[1][0][0])
    print("Equation left-line:", 0, "x +", coord[3][0][0])

    return Oval(img.shape[1], (x1, y1, r1), (x2, y2, r2), coord[1][0][0], coord[3][0][0])


def get_closest_point(x, y, oval):
    # x = int(round(float(x) / 0.01))
    # y = int(round(float(y) / 0.01))
    print("Coordinates in pixels: ", (x, y))
    coord_x = None
    coord_y = None
    distance = None
    if oval.lower_semicircle[1] > y > oval.upper_semicircle[1]:
        if x >= oval.width / 2:
            coord_x = oval.right_segment
            coord_y = y
            distance = np.abs(x - coord_x) * 0.01
            plt.scatter([coord_x], [coord_y], c='g', s=40)
            print(
                "The given point is closer to the right line at coord = ",
                (int(coord_x), int(coord_y)),
                "Distance in meters:",
                distance
            )
        else:
            coord_x = oval.left_segment
            coord_y = y
            distance = np.abs(x - coord_x) * 0.01
            plt.scatter([coord_x], [coord_y], c='g', s=40)
            print("The given point is closer to the left line at coord = ",
                  (int(coord_x), int(coord_y)),
                  "Distance in meters:",
                  distance
            )
    elif y <= oval.upper_semicircle[1]:
        distance = distance_to_the_circle(
            x,
            y,
            oval.upper_semicircle[0],
            oval.upper_semicircle[1],
            oval.upper_semicircle[2]
        ) * 0.01
        coord_x, coord_y = get_intercept_point(
            x,
            y,
            oval.upper_semicircle[0],
            oval.upper_semicircle[1],
            oval.upper_semicircle[2]
        )
        plt.scatter([coord_x], [coord_y], c='g', s=40)
        print(
            "The given point is closer to the upper circle at:",
            (int(coord_x), int(coord_y)),
            ", Distance in meters:",
            distance
        )
    elif y >= oval.lower_semicircle[1]:
        distance = distance_to_the_circle(
            x,
            y,
            oval.lower_semicircle[0],
            oval.lower_semicircle[1],
            oval.lower_semicircle[2]
        ) * 0.01
        coord_x, coord_y = get_intercept_point(
            x,
            y,
            oval.lower_semicircle[0],
            oval.lower_semicircle[1],
            oval.lower_semicircle[2]
        )
        plt.scatter([coord_x], [coord_y], c='g', s=40)
        print(
            "The given point is closer to the lower circle at:",
            (int(coord_x), int(coord_y)),
            ", Distance in meters:",
            distance
        )

    # return (coord_x * 0.01, coord_y * 0.01), distance
    return (coord_x, coord_y), distance


def perpendicular_point(proj_x, proj_y, lane_x, lane_y, dist):
    check_y = proj_y - lane_y
    check_x = proj_x - lane_x
    if check_x == 0:
        if check_y > 0:
            return proj_x + dist, proj_y
        else:
            return proj_x - dist, proj_y
    if check_y == 0:
        if check_x > 0:
            return proj_x, proj_y - dist
        else:
            return proj_x, proj_y + dist
    aux = dist / (np.sqrt((lane_y - proj_y)**2 + (proj_x - lane_x)**2))
    if check_x < 0 and check_y < 0:
        perp_x = proj_x + (aux * (proj_x - lane_x))
        if proj_x < perp_x < proj_x + 10:
            perp_x += dist
        if proj_x - 10 < perp_x < proj_x:
            perp_x -= dist
        perp_y = proj_y + (aux * (lane_y - proj_y))
        if proj_y < perp_y < proj_y + 10:
            perp_y += dist
        if proj_y - 10 < perp_y < proj_y:
            perp_y -= dist
        return perp_x, perp_y
    perp_x = proj_x - (aux * (proj_x - lane_x))
    if proj_x < perp_x < proj_x + 10:
        perp_x += dist
    if proj_x - 10 < perp_x < proj_x:
        perp_x -= dist
    perp_y = proj_y - (aux * (lane_y - proj_y))
    if proj_y < perp_y < proj_y + 10:
        perp_y += dist
    if proj_y - 10 < perp_y < proj_y:
        perp_y -= dist
    return perp_x, perp_y


def get_world_coordinates(theta, trans_x, trans_y, point_x, point_y):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    transformation_matrix = np.array([[cos_theta, -sin_theta, 0, trans_x],
                                      [sin_theta,  cos_theta, 0, trans_y],
                                      [0,                  0, 1,       0],
                                      [0,                  0, 0,       1]])
    obst_vector = np.array([point_x, point_y, 0, 1])
    obst_world = np.dot(transformation_matrix, obst_vector)
    return obst_world


def main(argv):
    # x = int(round(float(argv[0]) / 0.01))
    # y = int(round(float(argv[1]) / 0.01))
    x = int(round(1.2 / 0.01))
    y = int(round(3.2 / 0.01))
    plt.scatter([y], [x], c='r', s=40)
    inner_track = calculate_shape(26)
    print(inner_track)
    outer_track = calculate_shape(29)
    print(outer_track)
    point_2, distance = get_closest_point(y, x, inner_track)

    # arg0 = int(round(float(argv[0]) / 0.01))
    # arg1 = int(round(float(argv[1]) / 0.01))
    point_3 = get_point_to_distance(inner_track, (y, x), point_2, 25)

    plt.scatter([point_3[0]], [point_3[1]], c='b', s=40)
    print("point3:", point_3)
    print("point2:", point_2)

    perp_point = perpendicular_point(point_3[0], point_3[1], point_2[0], point_2[1], 50)
    print("perp_point: ", perp_point)
    plt.scatter([perp_point[0]], [perp_point[1]], c='w', s=40)

    perp_point_2, distance = get_closest_point(perp_point[0], perp_point[1], inner_track)
    perp_point_3 = get_point_to_distance(inner_track, (perp_point[0], perp_point[1]), perp_point_2, 25)
    perp_point_np = np.array((perp_point_3[0], perp_point_3[1]))
    #
    plt.scatter([perp_point_3[0]], [perp_point_3[1]], c='pink', s=40)

    dist = np.linalg.norm(np.array((y, x)) - perp_point_np)
    print(dist)

    # coords, distance = get_closest_point(argv[0], argv[1], outer_track)
    plt.show()
    print("chao mundo")
    # x = 2
    # y = 2
    # ox = 2
    # oy = 3
    # theta = np.pi/2
    # print(get_world_coordinates(theta, x, y, ox, oy)[:2])


if __name__== "__main__":
  main(sys.argv[1:])
