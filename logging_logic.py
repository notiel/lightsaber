from pyquaternion import Quaternion

import graph
from quaternion_algs import slow_quatern_from_data, raw_quatern_from_data, madgwick_filtered
from utils import data_split

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
    # acc_data_log = list()
    # gyro_data_log = list()
    # detected_events = list()
    # # quatertnion_data = list()
    # quatertnion_data2 = list()
    # quatertnion_data3 = list()
    # q_data=[Quaternion()]
    # v_raw = (1, 0, 0)
    # v_filtered = (1, 0, 0)
    # # q_simple = Quaternion()
    # q_fast = Quaternion()
    # q_madj = Quaternion()
    # raw_log = list()
    # filtered_log = list()
    return created_storage


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