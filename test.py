import time
from obj_state.road_obj import *

@unique
class relation_state(Enum):
    """describ the relation between ego vehicle and other obj
    Params:
        none_overlap:
        one_overlap:
        all_overlap:
    """
    none_overlap = 0
    overlap_in_x = 1
    overlap_in_y = 2
    all_overlap = 3

if __name__ == "__main__":
    """obj_state.road_obj test"""
    # ## init some vars ##
    # size = (1., 2., 3.)
    # position = (2., 3., 4.)
    # orientation = (1., 0., 0., 0.)
    # linear = (1., 2., 3.)
    # angular = 3. ## default in axis z
    # t = time.time()
    # obj_type = obj_type.vehicle
    # obj = road_obj()
    #
    # ## set attr ##
    # obj.set_size(size)
    # obj.set_position(position)
    # obj.set_linear(linear)
    # obj.set_orientation(orientation)
    # obj.set_angular(angular)
    # obj.set_time_stamp(t)
    # obj.set_obj_type(obj_type)
    #
    # ## get attr ##
    # s = obj.get_size()
    #
    # pt = obj.get_position()
    # l = obj.get_linear()
    # ps1 = obj.get_orientation(format = 'rpy')
    # ps2 = obj.get_orientation(format = 'xyzw')
    # a = obj.get_angular()
    # t = obj.get_time_stamp()
    # type = obj.get_type()
