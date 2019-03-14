"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
provides a tool to assess the degree of safety when driving.

Author：Team Li
"""
from enum import Enum, unique
import msgs.scene_msg as scene_m
from obj_state import ego_vehicle as ego_v
from obj_state import road_obj as road_o
import math

# safety degree #
@unique
class safety_degree(Enum):
    """describ the safety degree
    """
    safe = 0
    attentive = 1
    dangerous = 2


def _assess_one_obj(ego_vehicle, road_obj):
    """assess the road object safety degree for ego vehicle
    Args:
        ego_vehicle: a class in obj_state.ego_vehicel
        road_obj: a class in obj_state.road_obj
    """
    ## assert ##
    assert type(ego_vehicle) == ego_v.ego_vehicle
    assert type(road_obj) == road_o.road_obj

    ego_x, ego_y, ego_z = ego_vehicle.get_position()
    other_x, other_y, other_z = road_obj.get_position()

    ego_v_x, ego_v_y, ego_v_z = ego_vehicle.get_linear()
    other_v_x, other_v_y, other_v_z = road_obj.get_linear()

    ttc_x = (ego_x - other_x)/(ego_v_x - other_v_x)
    ttc_y = (ego_y - other_y)/(ego_v_y - other_v_y)