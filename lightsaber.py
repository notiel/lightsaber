# import serial
from collections import namedtuple
from matplotlib import pyplot as plot

# HIT_HIGH_A = 0
import events
import graph
from math import pi, sin, cos
from pyquaternion import Quaternion


def get_new_states(acc_data: iter, gyro_data: iter, parameters: dict, data: str, time: int, actions: dict) -> dict:
    """
    :param acc_data: queue with last acc ten measurements
    :param gyro_data: queue with last ten gyro measurements
    :param parameters: parameters of system state for current moment
    :param levels: constants to compare angular velocity and acceleration with
    :param data: new data from IMU
    :param time: current time counter
    :param actions:  current state of each action (swing, spin, clash, stab)
    :return: new actions state
    """
    accel, gyro = data_split(data)
    acc_data.append(accel)
    gyro_data.append(gyro)
    a_curr = sum([accel[i] * accel[i] for i in range(3)])
    w_curr = sum([gyro[i] * gyro[i] for i in [1, 2]])
    events.update_acc_data(parameters, actions, a_curr, time)
    events.update_gyro_data(parameters, actions, w_curr, time)
    actions['hit'] = events.check_hit_with_accelerometer_and_change(acc_data, time, parameters, actions['hit'])
    if time > 10:
        actions['swing'] = events.check_new_swing(gyro_data, time, parameters, actions['swing'])
    if not actions['stab']:
        actions['stab'] = events.check_stab(acc_data, gyro_data, time, parameters)
    parameters['w_prev'] = w_curr
    return actions


def data_split(data):
    """
    splits data flow from specific strings into two lists
    >>> data_split ("3909 304 -1591; 82 11 -39")
    ([3909, 304, -1591], [82, 11, -39])
    """
    data = data.split(';')
    accel = list(map(int, data[0].split()))
    gyro = list(map(int, data[1].split()))
    return accel, gyro


def gyro_to_rad(gx, gy, gz):
    """
    convert gyro data to radians, by Camill
    Could be wrong
    """
    #TODO: request assistance
    g_s = 2000.0 / 32768 / 180 * pi  # GYRO_SCALE
    return gx * g_s, gy * g_s, gz * g_s


def quatern_from_data(data: str, delay: float) -> Quaternion:
    """
    expected that data contains radians
    :param data:
    :param delay:
    :return:
    """
    _, (dgx, dgy, dgz) = data_split(data)
    gx, gy, gz = gyro_to_rad(dgx, dgy, dgz)
    # now it is time to use SO-driven-developement
    # credits to https://stackoverflow.com/a/28757303/2730579
    Vector3 = namedtuple('Vector3', 'x y z')
    # created a vector with set delay but halved
    hv = Vector3(x=float(gx * delay) / 2, y=float(gy * delay) / 2, z=float(gz * delay) / 2)
    w: float = cos(hv.x) * cos(hv.y) * cos(hv.z) + sin(hv.x) * sin(hv.y) * sin(hv.z)
    x: float = sin(hv.x) * cos(hv.y) * cos(hv.z) - cos(hv.x) * sin(hv.y) * sin(hv.z)
    y: float = cos(hv.x) * sin(hv.y) * cos(hv.z) + sin(hv.x) * cos(hv.y) * sin(hv.z)
    z: float = cos(hv.x) * cos(hv.y) * sin(hv.z) - sin(hv.x) * sin(hv.y) * cos(hv.z)
    return Quaternion(w, x, y, z)


def main():
    # acc_data = deque(maxlen=10)
    # gyro_data = deque(maxlen=10)
    acc_data = list()
    gyro_data = list()

    parameters = {"w_prev": 0, 'a_high': 0, 'w_rising': 0, 'w_low': 0, 'a_start': -1, 'w_start': -1, 'hit_start': -1,
                  'stab_start': -1, 'a_swing': 0,
                  'a_stab_start': -1, 'a_stab': 0, 'w_low_start': -1, 'swing_starts': [], 'hit_starts': [],
                  'stab_starts': []}
    actions = {'spin': 0, 'swing': 0, 'hit': 0, 'stab': 0}
    time = 0
    f = open("res_data.txt")
    detected_events = list()
    quatertnion_data = list()
    q_simple = Quaternion()

    for data in f:
        time += 1
        actions = get_new_states(acc_data, gyro_data, parameters, data, time, actions)
        q_simple = q_simple * quatern_from_data(data, delay=0.001) # this appends non-filtered quaternions from gyro
        detected_events.append(dict(actions))
        quatertnion_data.append(q_simple)


    # graph.plot_swings(gyro_data, detected_events)
    graph.plot_quatern_wx(gyro_data, quatertnion_data)
    graph.plot_quatern_yz(gyro_data, quatertnion_data)
    plot.show()

    # print("Swing starts: %s" % " ".join(list(map(str, parameters['swing_starts']))))
    # print("Number of swings %s" % len(parameters['swing_starts']))
    # print("Hit starts: %s" % " ".join(list(map(str, parameters['hit_starts']))))
    # print("Number of hits %s" % len(parameters['hit_starts']))
    # print("Stab starts: %s" % " ".join(list(map(str, parameters['stab_starts']))))
    # print("Number of stabs %s" % len(parameters['stab_starts']))


if __name__ == '__main__':
    main()
