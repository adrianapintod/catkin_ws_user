import matplotlib.pyplot as plt
import numpy as np
import math
import xml.etree.ElementTree as ET
from sklearn import linear_model

max_left = None
max_right = None
slope_steer = None
intercept_steer = None

def ransac_method(commands):
    """
    Get the linear model of lines
    :param commands: dictionary of commands and corresponding angels
    :return: parameters m and b of the linear model
    """

    ransac = linear_model.RANSACRegressor()
    
    # We need to fing the corresponding commands (y-axis) for function
    angles_x = []
    commands_y = []
    
    for cmd, angle in commands.items():
        commands_y.append([cmd])
        angles_x.append([angle])
    
    ransac.fit(angles_x, commands_y)
    b = ransac.estimator_.intercept_
    m = ransac.estimator_.coef_

    return m, b

def get_actuator_command(angle):
    if angle < max_left:
        angle = max_left
    elif angle > max_right:
        angle = max_right
    
    # convert deg to rad
    angle = math.radians(angle)
    
    command = slope_steer * angle + intercept_steer
    return int(command)

#=========================
#Parse the XML file
def parse_xml(file_name):
    tree = ET.parse(file_name)
    root = tree.getroot()
    items = root.findall("./myPair/item")
    
    commands = {int(i.find('command').text) : float(i.find('steering').text) for i in items}
    print(commands)
    
    return commands

def calibrate_steer():
    file_name = '/home/oleksandra/Documents/catkin_ws_user/src/assignment7_line_detection_pd_control/src/SteerAngleActuator_125.xml'
    commands = parse_xml(file_name)
    
    global slope_steer, intercept_steer
    slope_steer, intercept_steer = ransac_method(commands)
    print("Steering line 1: y = %fx + %f" % (slope_steer, intercept_steer))

    # define borders
    global max_left, max_right
    max_left = math.degrees((179-intercept_steer)/slope_steer)
    max_right = math.degrees(-intercept_steer/slope_steer)
