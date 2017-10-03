# import serial
from collections import deque

# HIT_HIGH_A = 0
import events
from matplotlib import pyplot as plot


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
    data = data.split(';')
    accel = list(map(int, data[0].split()))
    acc_data.append(accel)
    gyro = list(map(int, data[1].split()))
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
    for data in f:
        time += 1
        actions = get_new_states(acc_data, gyro_data, parameters, data, time, actions)
        detected_events.append(dict(actions))

    plot.figure()
    ox = [x for x in range(0, len(acc_data))]
    for label in ['acc_x', 'acc_y', 'acc_z']:
        index = ['acc_x', 'acc_y', 'acc_z'].index(label)
        dots = [y[index] for y in acc_data]
        plot.plot(ox, dots, label=label)
    # swings:
    print([str(x) for x in detected_events if x['swing']])
    swing_stats = [y['swing'] for y in detected_events]
    swing_dots = [10000 if state else 0 for state in swing_stats]
    plot.plot(ox, swing_dots, 'black', label='swingsers', )

    plot.xlabel('ms')
    plot.ylabel('acc_data')
    plot.legend()
    plot.show()

    print("Swing starts: %s" % " ".join(list(map(str, parameters['swing_starts']))))
    print("Number of swings %s" % len(parameters['swing_starts']))
    print("Hit starts: %s" % " ".join(list(map(str, parameters['hit_starts']))))
    print("Number of hits %s" % len(parameters['hit_starts']))
    print("Stab starts: %s" % " ".join(list(map(str, parameters['stab_starts']))))
    print("Number of stabs %s" % len(parameters['stab_starts']))


if __name__ == '__main__':
    main()
