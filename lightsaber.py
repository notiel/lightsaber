# import serial
from collections import namedtuple
from matplotlib import pyplot as plot

# HIT_HIGH_A = 0
import events
import graph
from math import pi, sin, cos, sqrt
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


def data_split(data: str) -> (list, list):
    """
    splits data flow from specific strings into two lists
    >>> data_split ("3909 304 -1591; 82 11 -39")
    ([3909, 304, -1591], [82, 11, -39])
    """
    data = data.split(';')
    accel = list(map(int, data[0].split()))
    gyro = list(map(int, data[1].split()))
    return accel, gyro


def gyro_to_rad(gx: int, gy: int, gz: int) -> (float, float, float):
    """
    convert gyro data to radians, by Camill
    Could be wrong
    """
    # TODO: request assistance
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
    # SLOW!! could precache sin and cosines, to speedup 4x
    w: float = cos(hv.x) * cos(hv.y) * cos(hv.z) + sin(hv.x) * sin(hv.y) * sin(hv.z)
    x: float = sin(hv.x) * cos(hv.y) * cos(hv.z) - cos(hv.x) * sin(hv.y) * sin(hv.z)
    y: float = cos(hv.x) * sin(hv.y) * cos(hv.z) + sin(hv.x) * cos(hv.y) * sin(hv.z)
    z: float = cos(hv.x) * cos(hv.y) * sin(hv.z) - sin(hv.x) * sin(hv.y) * cos(hv.z)
    return Quaternion(w, x, y, z)


def fast_quatern_from_data(prev_q: Quaternion, data: str, delay: float) -> Quaternion:
    """
    expected that data contains radians
    :param data:
    :param delay:
    :return:
    """
    _, (dgx, dgy, dgz) = data_split(data)
    gx, gy, gz = gyro_to_rad(dgx, dgy, dgz)
    w, x, y, z = prev_q.elements
    # this one is using direct quaternion calculation.
    # stolen from MadgwickAHRS.cs
    # // Compute rate of change of quaternion
    qDot1 = 0.5 * (-x * gx - y * gy - z * gz)
    qDot2 = 0.5 * (w * gx + y * gz - z * gy)
    qDot3 = 0.5 * (w * gy - x * gz + z * gx)
    qDot4 = 0.5 * (w * gz + x * gy - y * gx)
    # // Integrate to yield quaternion
    w += qDot1 * delay
    x += qDot2 * delay
    y += qDot3 * delay
    z += qDot4 * delay
    # // normalise quaternion
    norm = 1.0 / sqrt(w * w + x * x + y * y + z * z)
    return Quaternion(w * norm, x * norm, y * norm, z * norm)


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
    #quatertnion_data = list()
    quatertnion_data2 = list()
    #q_simple = Quaternion()
    q_fast = Quaternion()

    for data in f:
        time += 1
        actions = get_new_states(acc_data, gyro_data, parameters, data, time, actions)
        #q_simple = q_simple * quatern_from_data(data, delay=0.001)  # this appends non-filtered quaternions from gyro
        q_fast = fast_quatern_from_data(q_fast, data, delay=0.001)  # this obtains fast quaternion from gyro
        detected_events.append(dict(actions))
        #quatertnion_data.append(q_simple)
        quatertnion_data2.append(q_fast)

    # graph.plot_swings(gyro_data, detected_events)
    graph.plot_quatern_wx(gyro_data, quatertnion_data2)
    graph.plot_quatern_yz(gyro_data, quatertnion_data2)
    plot.show()

    # print("Swing starts: %s" % " ".join(list(map(str, parameters['swing_starts']))))
    # print("Number of swings %s" % len(parameters['swing_starts']))
    # print("Hit starts: %s" % " ".join(list(map(str, parameters['hit_starts']))))
    # print("Number of hits %s" % len(parameters['hit_starts']))
    # print("Stab starts: %s" % " ".join(list(map(str, parameters['stab_starts']))))
    # print("Number of stabs %s" % len(parameters['stab_starts']))


if __name__ == '__main__':
    main()
