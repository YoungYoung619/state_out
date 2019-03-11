import time
from obj.road_obj import *

if __name__ == "__main__":
    ## init some vars ##
    size = (1., 2., 3.)
    position = (2., 3., 4.)
    orientation = (1., 0., 0., 0.)
    linear = (1., 2., 3.)
    angular = 3. ## default in axis z
    t = time.time()
    obj_type = obj_type.vehicle
    obj = road_obj()

    ## set attr ##
    obj.set_size(size)
    obj.set_position(position)
    obj.set_linear(linear)
    obj.set_orientation(orientation)
    obj.set_angular(angular)
    obj.set_time_stamp(t)
    obj.set_obj_type(obj_type)

    ## get attr ##
    s = obj.get_size()

    pt = obj.get_position()
    l = obj.get_linear()
    ps1 = obj.get_orientation(format = 'rpy')
    ps2 = obj.get_orientation(format = 'xyzw')
    a = obj.get_angular()
    t = obj.get_time_stamp()
    type = obj.get_type()
    pass
