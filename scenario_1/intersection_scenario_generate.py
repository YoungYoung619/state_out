"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :
generate two vehicle at a specefic intersection

Author：Team Li
"""
import sys, os, glob, random, threading, time
from situation_assessment import _assess_one_obj_threat_score
from situation_assessment import _score_2_threat_degree
from situation_assessment import safety_degree
from obj_state import ego_vehicle as ego_v
from obj_state import road_obj as road_o
from scenario_1 import config
try:
    sys.path.append(config.carla_egg_file)
    import carla
except:
    raise ImportError('Please check your carla file')
from carla_utils.logging import logger
from carla_utils.world_ops import *

import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate

################################################
########## whether use collision avoidance #####
###############################################
collision_avoidance = True


def get_host(world):
    actors = world.get_actors().filter('vehicle*')
    for actor in actors:
        if actor.attributes['role_name'] == 'host_vehicle':
            return actor

    raise ValueError('no host in world')

def get_other(world):
    actors = world.get_actors().filter('vehicle*')
    for actor in actors:
        if actor.attributes['role_name'] == 'other_vehicle':
            return actor

    raise ValueError('no other in world')


def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = np.linspace(-4, 4, 500)
    y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """
    import numpy as np
    from math import factorial

    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order + 1)
    half_window = (window_size - 1) // 2
    # precompute coefficients
    b = np.mat([[k ** i for i in order_range] for k in range(-half_window, half_window + 1)])
    m = np.linalg.pinv(b).A[deriv] * rate ** deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs(y[1:half_window + 1][::-1] - y[0])
    lastvals = y[-1] + np.abs(y[-half_window - 1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve(m[::-1], y, mode='valid')


def plot_threat_curve(thread_record_d, thread_record_a, thread_record_s):
    title_font_dict = {'family': 'Times New Roman',
                       'color':  'black',
                       'weight': 'normal',
                       'size': 30}

    axis_font_dict = {'family': 'Times New Roman',
                       'color':  'black',
                       'weight': 'normal',
                       'size': 28}
    legend_font = {'family': 'Times New Roman',
                   'weight': 'normal',
                   'size': 28,
             }

    thread_record_s = np.array(thread_record_s)
    thread_record_a = np.array(thread_record_a)
    thread_record_d = np.array(thread_record_d)
    end = thread_record_s[:, 0][len(thread_record_s[:, 0]) - 1]

    step = 4
    thread_record_s = thread_record_s[0:int(end):step]
    thread_record_a = thread_record_a[0:int(end):step]
    thread_record_d = thread_record_d[0:int(end):step]

    # plt.plot(thread_record_s[:, 0] / 10., thread_record_s[:, 1], color="green")
    # plt.plot(thread_record_a[:, 0] / 10., thread_record_a[:, 1], color="orange")
    # plt.plot(thread_record_d[:, 0] / 10., thread_record_d[:, 1], color="red")
    # plt.ylim(0, 1.1)
    # plt.show()
    xnew = np.linspace(0, (end - step - 1) / 10, 200)
    func_s = interpolate.interp1d(thread_record_s[:, 0] / 10., thread_record_s[:, 1], kind='slinear')
    func_a = interpolate.interp1d(thread_record_a[:, 0] / 10., thread_record_a[:, 1], kind='slinear')
    func_d = interpolate.interp1d(thread_record_d[:, 0] / 10., thread_record_d[:, 1], kind='slinear')

    ynew_s = func_s(xnew)
    ynew_a = func_a(xnew)
    ynew_d = func_d(xnew)

    # func_s_ = interpolate.interp1d(xnew, ynew_s, kind='cubic')
    # func_a_ = interpolate.interp1d(xnew, ynew_a, kind='cubic')
    # func_d_ = interpolate.interp1d(xnew, ynew_d, kind='cubic')
    #
    # xnew = np.arange(0, end/10, 0.01)
    # ynew_s = func_s_(xnew)
    # ynew_a = func_a_(xnew)
    # ynew_d = func_d_(xnew)

    # plt.plot(xnew, ynew_s, color="green")
    # plt.plot(xnew, ynew_a, color="orange")
    # plt.plot(xnew, ynew_d, color="red")
    # plt.ylim(0, 1.1)
    # plt.show()

    ynew_s = savitzky_golay(ynew_s, 21, order=2)
    ynew_a = savitzky_golay(ynew_a, 21, order=2)
    ynew_d = savitzky_golay(ynew_d, 21, order=2)

    plt.plot(xnew, np.clip(ynew_s, 0., 1.), color="green", label="Safe", linewidth=2)
    plt.plot(xnew, np.clip(ynew_a, 0., 1.), color="orange", label="Attentive", linewidth=2)
    plt.plot(xnew, np.clip(ynew_d, 0., 1.), color="red", label="Dangerous", linewidth=2)

    plt.title('Likehood of Different Threat Degree', fontdict=title_font_dict)
    plt.xlabel('Time (s)', fontdict=axis_font_dict)
    plt.ylabel('Likehood', fontdict=axis_font_dict)
    plt.tick_params(labelsize=20)
    plt.legend(prop=legend_font)
    plt.ylim(0., 1.)
    left, right = plt.xlim()
    plt.xlim(0., right)
    plt.show()

def control_host(host_vehicle):
    """ a controller control the host, if emergency event(Collision) would be happen, it would prevent it."""
    def pd_control_for_collision(velocity, distance_collision_host, last_error):
        """a simple PD controller of host vehicle for coliision purpose"""
        k_p = 1
        k_d = 0.5

        other = get_other(world)
        other_pos = other.get_location()
        distance_collision = other_pos.distance(carla.Location(x=-77.5, y=-3., z=pos.z))
        o_velocity = other.get_velocity()
        o_v = math.sqrt(o_velocity.x**2+o_velocity.y**2)
        ttc = distance_collision/(o_v+1e-10)
        target_v = distance_collision_host/(ttc+1e-10)

        error = target_v - math.sqrt(velocity.x**2+velocity.y**2)
        d_error = error - last_error

        throttle = k_p*error + k_d*d_error

        return max(min(throttle, 1.), 0.), error

    last_error = 0.
    time_step = 0
    emergency = False
    thread_record_d = []
    thread_record_a = []
    thread_record_s = []
    while True:
        ## record info
        pos = host_vehicle.get_location()
        velocity = host_vehicle.get_velocity()
        acc = host_vehicle.get_acceleration()
        # logger.info('host_vehicle velocity:' + str(velocity) + ' acc:' + str(acc))

        pos = host_vehicle.get_location()
        distance_collision = pos.distance(carla.Location(x=-77.5, y=-3., z=pos.z))
        # logger.info('remaining distance:' + str(distance_collision))

        ## assess the situation
        ego_v_state = ego_v.ego_vehicle()
        ego_v_state.set_position(position=(pos.x, pos.y, pos.z))
        ego_v_state.set_linear(linear=(velocity.x, velocity.y, velocity.z))
        ego_v_state.set_size(size=(1.3, 1.5, 3.))

        other = get_other(world)
        other_pos = other.get_location()
        other_velocity = other.get_velocity()
        road_obj_state = road_o.road_obj()
        road_obj_state.set_position(position=(other_pos.x, other_pos.y, other_pos.z))
        road_obj_state.set_linear(linear=(other_velocity.x, other_velocity.y, other_velocity.z))
        road_obj_state.set_size(size=(1.3, 1.5, 3.))
        score = _assess_one_obj_threat_score(ego_v_state, road_obj_state)
        degree = _score_2_threat_degree(score)
        thread_record_d.append([time_step, score[0]])
        thread_record_a.append([time_step, score[1]])
        thread_record_s.append([time_step, score[2]])
        if degree == safety_degree.dangerous and score[0] >= 0.8:
            emergency = True
        # logger.info('threat degree for other vehicle:'+str(degree))
        ## assess the situation

        ## control
        if collision_avoidance:
            if not emergency:
                throttle, last_error = pd_control_for_collision(velocity, distance_collision, last_error)
                control = carla.VehicleControl(throttle=throttle)
            else:
                control = carla.VehicleControl(brake=1.)
                v = math.sqrt(velocity.x**2+velocity.y**2)
                if v < 0.1:
                    logger.info('Stop testing...')
                    i = 0
                    while (i<10):
                        thread_record_d.append([time_step, score[0]])
                        thread_record_a.append([time_step, score[1]])
                        thread_record_s.append([time_step, score[2]])
                        time_step += 1
                        i += 1
                        time.sleep(0.1)
                    break
        else:
            throttle, last_error = pd_control_for_collision(velocity, distance_collision, last_error)
            control = carla.VehicleControl(throttle=throttle)
        host_vehicle.apply_control(control)


        # if distance_collision > 10:
        #     control = carla.VehicleControl(throttle=1.)
        #     host_vehicle.apply_control(control)
        # else:
        #     control = carla.VehicleControl(brake=1.)
        #     host_vehicle.apply_control(control)
        time.sleep(0.1)
        time_step += 1

    plot_threat_curve(thread_record_d, thread_record_a, thread_record_s)



def control_other(other_vehicle):
    while True:
        # ## record info
        # velocity = other_vehicle.get_velocity()
        # acc = other_vehicle.get_acceleration()
        # logger.info('other_vehicle velocity:' + str(velocity) + ' acc:' + str(acc))

        # pos = other_vehicle.get_location()
        # distance_collision = pos.distance(carla.Location(x=-77.5, y=-3., z=pos.z))
        # logger.info('remaining distance:' + str(distance_collision))

        throttle = random.uniform(other_vehicle_speed_range[0], other_vehicle_speed_range[1])
        control = carla.VehicleControl(throttle=throttle)
        other_vehicle.apply_control(control)
        time.sleep(0.1)


if __name__ == '__main__':
    ##############################
    ####### general config #######
    other_vehicle_init_pos = random.randint(40, 90)
    other_vehicle_speed_range = (random.uniform(0.6, 0.9), random.uniform(0.9, 1.))
    ##############################

    #### carla world init ####
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)  # seconds
    logger.info('Carla connect success...')

    logger.info('Carla world initing...')
    world = client.get_world()

    destroy_all_actors(world)

    ## vehicle blueprint
    blueprints = world.get_blueprint_library().filter('vehicle.nissan.micra')
    blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]

    ## host vehicle settings
    host_vehicle_bp = random.choice(blueprints)
    if host_vehicle_bp.has_attribute('color'):
        color = random.choice(host_vehicle_bp.get_attribute('color').recommended_values)
        host_vehicle_bp.set_attribute('color', color)
        host_vehicle_bp.set_attribute('role_name', 'host_vehicle')
    transform = carla.Transform(carla.Location(x=-39.0, y=-3., z=1.8), carla.Rotation(pitch=0., yaw=180., roll=0.))
    try_spawn_at(world, host_vehicle_bp, transform, autopilot=False)

    ## other vehicle settings
    other_vehicle_bp = random.choice(blueprints)
    if other_vehicle_bp.has_attribute('color'):
        color = random.choice(other_vehicle_bp.get_attribute('color').recommended_values)
        other_vehicle_bp.set_attribute('color', color)
        other_vehicle_bp.set_attribute('role_name', 'other_vehicle')
    transform = carla.Transform(carla.Location(x=-77.75, y=float(other_vehicle_init_pos), z=1.8),
                                carla.Rotation(pitch=0., yaw=-90., roll=0.))
    try_spawn_at(world, other_vehicle_bp, transform, autopilot=False)
    time.sleep(1) ## waiting carla synchronous

    logger.info('host vehicle location: '+str(get_host(world).get_location()))
    logger.info('other vehicle location: '+str(get_other(world).get_location()))
    logger.info('other vehicle throttle range: '+str(other_vehicle_speed_range))
    logger.info('The test will start in 1 seconds, notice the Carla screen!!')
    time.sleep(1)

    logger.info('Start testing...')
    control_host_t = threading.Thread(target=control_host, args=(get_host(world),))
    control_other_t = threading.Thread(target=control_other, args=(get_other(world),))
    control_host_t.start()
    control_other_t.start()

    while True:
        pass

