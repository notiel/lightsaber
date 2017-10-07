# import serial
from collections import deque

from matplotlib import pyplot as plot
from pyquaternion import Quaternion

# HIT_HIGH_A = 0
import events
import graph
from quaternion_algs import slow_quatern_from_data, madgwick_filtered, raw_quatern_from_data


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


# TODO fix interface
# Probably should migrate to OOP
# or elif
# BLAME ME! Ariksu
def acc_from_data(_, data):
    return data_split(data)[0]


def gyro_from_data(_, data):
    return data_split(data)[1]


def actions_mock(_, __, actions):
    return actions


def orientation_evo(qlist):
    vlist = [q.rotate((1, 0, 0)) for q in qlist]
    graph.plot_vector_evo(vlist)
    pass


def create_data_storage(config):
    initial = {
        'collect_acc_data': [[0, 0, 0]],
        'collect_gyro_data': [[0, 0, 0]],
        'collect_events': [{'spin': 0, 'swing': 0, 'hit': 0, 'stab': 0}],
        'slow_orientation': [Quaternion()],
        'fast_orientation': [Quaternion()],
        'filtered_orientation': [Quaternion()],
    }
    created_storage = {key: initial[key] for key in config if key in initial}
    return created_storage


def main():
    acc_data = deque(maxlen=10)
    gyro_data = deque(maxlen=10)
    acc_data_log = list()
    gyro_data_log = list()

    parameters = {"w_prev": 0, 'a_high': 0, 'w_rising': 0, 'w_low': 0, 'a_start': -1, 'w_start': -1, 'hit_start': -1,
                  'stab_start': -1, 'a_swing': 0,
                  'a_stab_start': -1, 'a_stab': 0, 'w_low_start': -1, 'swing_starts': [], 'hit_starts': [],
                  'stab_starts': []}
    actions = {'spin': 0, 'swing': 0, 'hit': 0, 'stab': 0}
    time = 0
    f = open("res_data.txt")
    config = {
        'delay': 0.001,
        'beta': 0.03,
        'collect_acc_data': True,
        'collect_gyro_data': True,
        'collect_events': True,
        'plot_swing': False,
        'slow_orientation': False,
        'fast_orientation': True,
        'filtered_orientation': True,
    }
    log_storage = create_data_storage(config)
    detected_events = list()
    # quatertnion_data = list()
    quatertnion_data2 = list()
    quatertnion_data3 = list()
    q_data=[Quaternion()]
    v_raw = (1, 0, 0)
    v_filtered = (1, 0, 0)
    # q_simple = Quaternion()
    q_fast = Quaternion()
    q_madj = Quaternion()
    raw_log = list()
    filtered_log = list()

    for data in f:
        time += 1
        actions = get_new_states(acc_data, gyro_data, parameters, data, time, actions)
        calculate_and_collect(data, config, log_storage, actions=actions)


    plot_collected(config, log_storage)
    plot.show()

    # print("Swing starts: %s" % " ".join(list(map(str, parameters['swing_starts']))))
    # print("Number of swings %s" % len(parameters['swing_starts']))
    # print("Hit starts: %s" % " ".join(list(map(str, parameters['hit_starts']))))
    # print("Number of hits %s" % len(parameters['hit_starts']))
    # print("Stab starts: %s" % " ".join(list(map(str, parameters['stab_starts']))))
    # print("Number of stabs %s" % len(parameters['stab_starts']))


def plot_collected(config, storage):
    acc_data_log = storage['collect_acc_data']
    gyro_data_log = storage['collect_gyro_data']
    plots = {
        'plot_swing': {'exec': graph.plot_swings,
                       'args': [gyro_data_log]},
        'slow_orientation': {'exec': orientation_evo,
                             'args': []},
        'fast_orientation': {'exec': orientation_evo,
                             'args': []},
        'filtered_orientation': {'exec': orientation_evo,
                                 'args': []},
    }
    for p_type in config:
        if config[p_type] is True and p_type in plots:
            plots[p_type]['exec'](storage[p_type], *plots[p_type]['args'])


    # graph.plot_swings(gyro_data, detected_events)
    # graph.plot_quatern_yz(gyro_data_log, quatertnion_data2)
    # graph.plot_quatern_yz(gyro_data_log, quatertnion_data3)
    # graph.plot_quaternion_evo(quatertnion_data2)
    # graph.plot_quaternion_evo(quatertnion_data3)
    # TODO resolve orientation vector regress
    # graph.plot_vector_evo(raw_log)
    # graph.plot_vector_evo(filtered_log)

    pass


def calculate_and_collect(data, config, storage, actions):
    delay = config['delay']
    beta = config['beta']
    vector = {
        'collect_acc_data': {'exec': acc_from_data,
                             'kw': {}},
        'collect_gyro_data': {'exec': gyro_from_data,
                              'kw': {}},
        'collect_events': {'exec': actions_mock,
                           'kw': {'actions': actions}},
        'slow_orientation': {'exec': slow_quatern_from_data,
                             'kw': {'delay': delay}},
        'fast_orientation': {'exec': raw_quatern_from_data,
                             'kw': {'delay': delay}},
        'filtered_orientation': {'exec': madgwick_filtered,
                                 'kw': {'delay': delay, 'beta': beta}},
    }
    for v_type in config:
        if config[v_type] is True and v_type in vector:
            log = storage[v_type]
            current = log[-1]
            new = vector[v_type]['exec'](current, data, **vector[v_type]['kw'])
            storage[v_type].append(new)

    # acc_data_log.append(acc_data[-1])
    # gyro_data_log.append(gyro_data[-1])
    # q_simple = q_simple * slow_quatern_from_data(data, delay=0.001)  # this appends non-filtered quaternions from gyro
    # q_fast = raw_quatern_from_data(q_fast, data, delay=0.001)  # this obtains fast quaternion from gyro
    # q_madj = madgwick_filtered(q_madj, data, delay=0.001, beta=0.1)  # this obtains madjwick-filtered quaternion
    # detected_events.append(dict(actions))
    # quatertnion_data2.append(q_fast)
    # raw_log.append(q_fast.rotate(v_raw))
    # quatertnion_data3.append(q_madj)
    # filtered_log.append(q_madj.rotate(v_filtered))
    pass


if __name__ == '__main__':
    main()
