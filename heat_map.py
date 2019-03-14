"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
A package wihch provides tools for generating pixel-level thermal maps that can be
used as states in reinforcement learning for autonomous vehicle.

Author：Team Li
"""

import numpy as np

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
        print('Warning: pls ensure direction is "x" or "y"')
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
        print('Warning: pls ensure direction is "x" or "y"')
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