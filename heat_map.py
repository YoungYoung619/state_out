"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
A package wihch provides tools for generating pixel-level thermal maps that can be
used as states in reinforcement learning for autonomous vehicle.

Author：Team Li
"""

import numpy as np
from msgs.log import logger

from obj_state import ego_vehicle as ego_v
from obj_state import road_obj as road_o
import math
from situation_assessment import _assess_one_obj_safety
from kp2hm_utils import heat_map


##########################
######### CONFIG #########
##########################
heat_map_size = (300, 300)      ##(h, w)
##########################

def get_pose(objs_info):
    """get pose heat map, the pose head map will include road_obj.position,
        road_obj.orientation and road_obj.size info.
    Args:
        objs_info: a list of road_obj.

    Return:
        A ndarray with the shape (heat_map_size[0], heat_map_size[1]),
        represents the pose heat map
    """
    p_heat_map = np.zeros(shape=(heat_map_size[0], heat_map_size[1]), dtype=np.float32)

    for obj_info in objs_info:
        if obj_info != None:
            position = obj_info.get_position() ## (x, y, z) ##
            orientation_e = obj_info.get_orientation(format='rpy')  ## euler angle (r, p, y) ##
            orientation_q = obj_info.get_orientation(format='xyzw')  ## quaternion (x, y, z, w) ##
            size = obj_info.get_size()      ## obj size (height, width, depth )
        else:
            ##sth error
            raise ValueError("Get a None obj_info..")

        #########################################
        #### to do (ceate the pose heat map) ####
        #########################################

    return p_heat_map

def get_linear(objs_info, direction):
    """get linear velocity heat map
    Args:
        objs_info: a list of road_obj.
        orientation: a str indicates the direction 'x' or 'y'

    Return:
        A ndarray with the shape (heat_map_size[0], heat_map_size[1]),
        represents the linear vel heat map
    """
    ## assert ##
    try:
        assert direction.lower() in ['x', 'y']
    except AssertionError:
        logger.error('Pls ensure direction is "x" or "y", but i get '+'"%s"'%(direction))
        exit(1)

    ## init heat map##
    vel_heat_map = np.zeros(shape=(heat_map_size[0], heat_map_size[1]), dtype=np.float32)

    ## creat heat map ##
    for obj_info in objs_info:
        if obj_info != None:
            position = obj_info.get_position()  ## (x, y, z) ##
            orientation_e = obj_info.get_pose(format='rpy')  ## euler angle (r, p, y) ##
            orientation_q = obj_info.get_pose(format='xyzw')  ## quaternion (x, y, z, w) ##
            size = obj_info.get_size()  ## obj size (height, width, depth )

            if direction.lower() == 'x':
                vel = obj_info.get_position().x     ## a float means velocity in direction x ##
            else:
                vel = obj_info.get_position().y     ## a float means velocity in direction y ##
        else:
            ##sth error
            raise ValueError("Get a None obj_info..")

        ####################################
        ## to do (ceate the vel heat map) ##
        ####################################

    return vel_heat_map


def get_angular(objs_info, axis='z'):
    """get angular rate heat map
        Args:
            objs_info: a list of road_obj.
            orientation: a str indicates the direction 'x' or 'y'

        Return:
            A ndarray with the shape (heat_map_size[0], heat_map_size[1]),
            represents the linear vel heat map
        """
    ## assert ##
    try:
        assert axis.lower() in ['x', 'y', 'z']
    except AssertionError:
        logger.error('Pls ensure direction is "x", "y", "z"')
        exit(1)

    ## init heat map##
    ang_heat_map = np.zeros(shape=(heat_map_size[0], heat_map_size[1]), dtype=np.float32)

    ## creat heat map ##
    for obj_info in objs_info:
        if obj_info != None:
            position = obj_info.get_position()  ## (x, y, z) ##
            orientation_e = obj_info.get_pose(format='rpy')  ## euler angle (r, p, y) ##
            orientation_q = obj_info.get_pose(format='xyzw')  ## quaternion (x, y, z, w) ##
            size = obj_info.get_size()  ## obj size (height, width, depth )

            if axis.lower() == 'z':
                ang = obj_info.get_angular().z  ## a float means angular in axis z ##
            elif axis.lower() == 'x':
                ang = obj_info.get_position().x  ## a float means angular in axis x ##
            else:
                ang = obj_info.get_position().y  ## a float means angular in axis y ##
        else:
            ##sth error
            raise ValueError("Get a None obj_info..")

        ####################################
        ## to do (ceate the ang heat map) ##
        ####################################

    return ang_heat_map


def produce_heat_map(ego, others, h_type, hm_size=(224, 224), consider_range=50):
    """produce heat map for each safety degree
    Args:
        ego: ego vehecle in carla
        others: other actor in carla
    Return:
        heat map
    """
    assert h_type in ['danger', 'attentive', 'safe']
    ego_location = ego.get_location()
    ego_location = (ego_location.x, ego_location.y, ego_location.z)
    ego_size = ego.bounding_box.extent
    ego_size = (ego_size.x, ego_size.y, ego_size.z)
    ego_velocity = ego.get_velocity()

    ego_velocity = (ego_velocity.x, ego_velocity.y, ego_velocity.z)

    t = ego.get_transform()
    f_v = t.get_forward_vector()

    cos_theta = (f_v.x*0 + f_v.y*1)/math.sqrt(f_v.x**2+ f_v.y**2)
    a = math.acos(cos_theta)
    if f_v.x > 0:
        a = -a

    r_matix = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    # points = []
    # sizes = []
    hms = []
    for vehicle in others:
        location = vehicle.get_location()
        location = (location.x, location.y, location.z)

        distance = math.sqrt((ego_location[0] - location[0]) ** 2 + (ego_location[1] - location[1]) ** 2)
        # print(vehicle, distance)
        # print(distance)
        if distance <= consider_range:
            size = vehicle.bounding_box.extent
            size = (size.x, size.y, size.z)
            velocity = vehicle.get_velocity()
            velocity = (velocity.x, velocity.y, velocity.z)

            ego_v_state = ego_v.ego_vehicle()
            ego_v_state.set_position(position=ego_location)
            ego_v_state.set_linear(linear=ego_velocity)
            ego_v_state.set_size(size=ego_size)

            road_obj_state = road_o.road_obj()
            road_obj_state.set_position(position=location)
            road_obj_state.set_linear(linear=velocity)
            road_obj_state.set_size(size=size)

            safety_degree = _assess_one_obj_safety(ego_vehicle=ego_v_state, road_obj=road_obj_state)
            max_index = np.argmax(np.array(safety_degree))
            if max_index == ['danger', 'attentive', 'safe'].index(h_type):
                relative_x = int(location[0] - ego_location[0])*(hm_size[1]//consider_range//2)
                relative_y = int(location[1] - ego_location[1])*(hm_size[0]//consider_range//2)

                point = np.matmul(np.array([relative_x, relative_y]), r_matix)

                point_x = min(hm_size[1]-1, max(-point[0] + hm_size[1]//2, 0))
                point_y = min(hm_size[0]-1, max(-point[1] + hm_size[0]//2, 0))

                size = vehicle.bounding_box.extent
                size = math.sqrt(size.x**2+size.y**2)

                hm = heat_map(hm_size, points=[[point_x, point_y]], sigma=size*2)
                hm *= safety_degree[max_index]
                hms.append(hm)
    if len(hms) > 0:
        hm = np.sum(np.array(hms), axis=0)
        return hm
    else:
        return np.zeros(hm_size)