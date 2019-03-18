"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
provides a tool to assess the degree of safety when driving.

Author：Team Li
"""
from enum import Enum, unique
import numpy as np
import math

import msgs.scene_msg as scene_m
from obj_state import ego_vehicle as ego_v
from obj_state import road_obj as road_o
from msgs.log import logger

# safety degree #
@unique
class safety_degree(Enum):
    """describ the safety degree
    """
    safe = 0
    attentive = 1
    dangerous = 2


@unique
class relation_state(Enum):
    """describ the relation between ego vehicle and other obj
    Params:
        none_overlap:
        overlap_in_x:
        overlap_in_y:
        all_overlap:
    """
    none_overlap = 0
    overlap_in_x = 1
    overlap_in_y = 2
    all_overlap = 3


def _assess_one_obj_safety(ego_vehicle, road_obj):
    """assess the road object safety degree for ego vehicle
    Args:
        ego_vehicle: a class in obj_state.ego_vehicel
        road_obj: a class in obj_state.road_obj
    """


    def judge_relation_state(ego_pos, ego_size, other_pos, other_size):
        """jugde the relation between ego vehicle and other obj
        Args:
            ego_pos: a tuple describe ego position, (x, y, z)
            ego_size: a tuple describe ego geometry size, (h, w, d)
            other_pos: a tuple describe other vehicle position, (x, y, z)
            other_size: a tuple describe other vehicle geometry size, (h, w, d)
        Return:
            a relation_state
        """
        ego_vehicle_radius = math.sqrt((ego_size[1]/2)**2 + (ego_size[2]/2)**2)
        other_obj_radius = math.sqrt((other_size[1]/2)**2 + (other_size[2]/2)**2)
        length = ego_vehicle_radius + other_obj_radius

        m_distance_x = math.fabs(ego_pos[0] - other_pos[0])
        m_distance_y = math.fabs(ego_pos[1] - other_pos[1])

        if m_distance_x <= length or m_distance_y <= length:
            if m_distance_x <= length and m_distance_y <= length:
                return relation_state.all_overlap
            elif m_distance_x <= length:
                return relation_state.overlap_in_x
            else:
                return relation_state.overlap_in_y
        else:
            return relation_state.none_overlap


    def time_to_collision_2d(ego_pos, ego_linear, ego_size, other_pos, other_linear, other_size, dim='xy'):
        """calculate TTC in x,y dimention, TTC is calculated as: TTC = (x[i-1] - x[i] - L) / (v[i] - v[i-1]),
            where i indicates the vehicle position which has the bigger velocity. When TTC > 0, it indicates
            that the remaining time in which two vehicles would collide. When TTC < 0, in indicates that it's
            impossible for two cars to collide.
        Args:
            ego_pos: a tuple describe ego position, (x, y, z)
            ego_linear: a tuple describe ego linear velocity in (x, y, z) dimention
            ego_size: a tuple describe ego geometry size, (h, w, d)
            other_pos: a tuple describe other vehicle position, (x, y, z)
            other_linear: a tuple describe ego linear velocity in (x, y, z) dimention
            other_size: a tuple describe other vehicle geometry size, (h, w, d)
            dim: a string indicates calculate TTC in which dimention, must be 'xy', 'y' or 'x'.
        Return:
            ttc_in_x: the remaining time in which two vehicles would collide in x dimention.
            ttc_in_y: the remaining time in which two vehicles would collide in y dimention.
        """
        ## assert ##
        assert dim in ['xy', 'x', 'y']

        ## length ##
        ego_vehicle_radius = math.sqrt((ego_size[1] / 2) ** 2 + (ego_size[2] / 2) ** 2)
        other_obj_radius = math.sqrt((other_size[1] / 2) ** 2 + (other_size[2] / 2) ** 2)
        length = ego_vehicle_radius + other_obj_radius

        ## 1e-5 is to avoid zero-divide error ##
        if ego_linear[0] >= other_linear[0]:
            ttc_in_x = (other_pos[0] - ego_pos[0] - length) / (ego_linear[0] - other_linear[0] + 1e-5)
        else:
            ttc_in_x = (ego_pos[0] - other_pos[0] - length) / (other_linear[0] - ego_linear[0] + 1e-5)

        if ego_linear[0] >= other_linear[0]:
            ttc_in_y = (other_pos[1] - ego_pos[1] - length) / (ego_linear[1] - other_linear[1] + 1e-5)
        else:
            ttc_in_y = (ego_pos[1] - other_pos[1] - length) / (other_linear[1] - ego_linear[1] + 1e-5)

        if dim == 'xy':
            return ttc_in_x, ttc_in_y
        elif dim == 'x':
            return ttc_in_x
        elif dim == 'y':
            return ttc_in_y
        else:
            raise ValueError("Imposible get here!!! Something must wrong!!!")


    def much_bigger(val, bigger_than):
        """ normally, the val should be a abs of subtraction value
        Return:
            if val >= bigger_than: True
            else: False
        """
        if val >= bigger_than:
            return True
        else:
            return False


    def time_to_escape_2d(ego_pos, ego_linear, ego_size, other_pos, other_linear, other_size, dim='xy'):
        """When the relation between ego vehicle and other obj is overlap, we need this indicator to
            indicate the remaining time in which the ego(or other obj) can escape the overlap area.
        Args:
            ego_pos: a tuple describe ego position, (x, y, z)
            ego_linear: a tuple describe ego linear velocity in (x, y, z) dimention
            ego_size: a tuple describe ego geometry size, (h, w, d)
            other_pos: a tuple describe other vehicle position, (x, y, z)
            other_linear: a tuple describe ego linear velocity in (x, y, z) dimention
            other_size: a tuple describe other vehicle geometry size, (h, w, d)
            dim: a string indicates calculate TTE in which dimention, must be 'xy', 'y' or 'x'.
        Return;
            TTE in specific dimention
        """
        ## length ##
        ego_vehicle_radius = math.sqrt((ego_size[1] / 2) ** 2 + (ego_size[2] / 2) ** 2)
        other_obj_radius = math.sqrt((other_size[1] / 2) ** 2 + (other_size[2] / 2) ** 2)
        length = ego_vehicle_radius + other_obj_radius

        ## 1e-5 is to avoid zero-divide error ##
        if ego_linear[0] >= other_linear[0]:
            tte_in_x = (other_pos[0] - ego_pos[0] + length) / (ego_linear[0] - other_linear[0] + 1e-5)
        else:
            tte_in_x = (ego_pos[0] - other_pos[0] + length) / (other_linear[0] - ego_linear[0] + 1e-5)

        if ego_linear[0] >= other_linear[0]:
            tte_in_y = (other_pos[1] - ego_pos[1] + length) / (ego_linear[1] - other_linear[1] + 1e-5)
        else:
            tte_in_y = (ego_pos[1] - other_pos[1] + length) / (other_linear[1] - ego_linear[1] + 1e-5)

        if dim == 'xy':
            return tte_in_x, tte_in_y
        elif dim == 'x':
            return tte_in_x
        elif dim == 'y':
            return tte_in_y
        else:
            raise ValueError("Imposible get here!!! Something must wrong!!!")


    ## assert ##
    assert type(ego_vehicle) == ego_v.ego_vehicle
    assert type(road_obj) == road_o.road_obj

    ## init safety degree score ##
    safety_score = np.zeros(shape=[len(list(safety_degree.__members__.values()))]) ##

    ego_pos = ego_vehicle.get_position()
    ego_linear = ego_vehicle.get_linear()
    ego_size = ego_vehicle.get_size()
    other_pos = road_obj.get_position()
    other_size = road_obj.get_size()
    other_linear = road_obj.get_linear()

    r_state = judge_relation_state(ego_pos, ego_size, other_pos, other_size)

    if r_state is relation_state.none_overlap:
        ttc_in_x, ttc_in_y = time_to_collision_2d(ego_pos, ego_linear, ego_size,
                                                  other_pos, other_linear, other_size)

        if ttc_in_x < 0. or ttc_in_y < 0.:
            ## safe ##
            pass
        else:
            ## maybe unsafe ##
            if much_bigger(math.fabs(ttc_in_y - ttc_in_x), 5.):
                ## safe ##
                pass
            else:
                ## safe, attentive or dagerous ##
                pass
            pass

    elif r_state is relation_state.overlap_in_x:
        tte_in_x = time_to_escape_2d(ego_pos, ego_linear, ego_size,
                                     other_pos, other_linear, other_size, dim='x')
        ttc_in_y = time_to_collision_2d(ego_pos, ego_linear, ego_size,
                                     other_pos, other_linear, other_size, dim='y')

        try:
            assert tte_in_x >= 0.
        except AssertionError:
            logger.warning('Get a negative tte val(%f) in x, something may be wrong!'%(tte_in_x))

        if ttc_in_y < 0.:
            ## safe ##
            pass
        else:
            if much_bigger(ttc_in_y - tte_in_x, 3.):
                ## safe ##
                pass
            elif much_bigger(tte_in_x - ttc_in_y, 5.):
                ## safe ##
                pass
            else:
                ## safe, attentive, dangerous ##
                pass
            pass
        ## to do ##

    elif r_state is relation_state.overlap_in_y:
        tte_in_y = time_to_escape_2d(ego_pos, ego_linear, ego_size,
                                     other_pos, other_linear, other_size, dim='y')
        ttc_in_x = time_to_collision_2d(ego_pos, ego_linear, ego_size,
                                        other_pos, other_linear, other_size, dim='x')

        ## to do ##
    elif r_state is relation_state.all_overlap:
        pass
    else:
        raise ValueError("Imposible get here!!! Something must wrong!!!")
    # ego_v_x, ego_v_y, ego_v_z = ego_vehicle.get_linear()
    # other_v_x, other_v_y, other_v_z = road_obj.get_linear()
    #
    # ttc_x = (ego_x - other_x)/(ego_v_x - other_v_x)
    # ttc_y = (ego_y - other_y)/(ego_v_y - other_v_y)


