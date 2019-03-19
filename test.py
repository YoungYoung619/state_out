import time
from obj_state.road_obj import *
import numpy as np
import matplotlib.pyplot as plt
import math
import mpl_toolkits.mplot3d


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

    """2d Gauss test"""
    # x, y = np.mgrid[-2:2:200j, -2:2:200j]
    # z = (1 / 2 * math.pi * 3 ** 2) * np.exp(-(x ** 2 + y ** 2) / 2 * 3 ** 2)
    # ax = plt.subplot(111, projection='3d')
    # ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='rainbow',
    #                 alpha=0.9)  # 绘面
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # plt.show()

    def gaussian_1d(x, mean, for_what, std=.5):
        """produce a val respresents the socre according the gaussian distribution.
        Args:
            x: a input val.
            mean: the mean of gaussian distribution.
            std: the std of gaussian distribution.
            for_what: should be one of safety_degree
        """

        def norm(x, mu, sigma):
            """normal gaussian function
            """
            pdf = math.exp(-((x - mu) ** 2) / (2 * sigma ** 2)) / (sigma * math.sqrt(2 * np.pi))
            return pdf

        assert for_what in list(safety_degree.__members__.values())

        if for_what is safety_degree.dangerous:
            if x < mean:
                score = norm(x=mean, mu=mean, sigma=std)
            else:
                score = norm(x=x, mu=mean, sigma=std)
        elif for_what is safety_degree.attentive:
            score = norm(x=x, mu=mean, sigma=std)
        elif for_what is safety_degree.safe:
            if x > mean:
                score = norm(x=mean, mu=mean, sigma=std)
            else:
                score = norm(x=x, mu=mean, sigma=std)
        else:
            raise ValueError("Error")

        return score


    xs = np.arange(0., 5., 0.01)
    y = []
    for x in xs:
        y.append(gaussian_1d(x, mean=0.15, for_what=safety_degree.dangerous))
    plt.plot(xs, y, label="dangerous", color='red')

    y = []
    for x in xs:
        y.append(gaussian_1d(x, mean=.3, for_what=safety_degree.attentive))
    plt.plot(xs, y, label="attentive", color='goldenrod')

    y = []
    for x in xs:
        y.append(gaussian_1d(x, mean=.4, for_what=safety_degree.safe))
    plt.plot(xs, y, label="safe", color='green')

    plt.legend()
    plt.title('Time distribution')
    plt.xlabel('Time')
    plt.ylabel('Probability')
    # 输出
    plt.show()
