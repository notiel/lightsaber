from collections import deque

import events


def get_new_states(acc_data: iter, gyro_data: iter, parameters: dict, data: str, time: int, actions: dict) -> dict:
    """
    :param queue: queue with last ten measurements
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
    w_curr = sum([gyro[i] * gyro[i] for i in range(3)])
    events.update_acc_data(parameters, actions, a_curr, time)
    events.update_gyro_data(parameters, actions, w_curr, time)
    actions['hit'] = events.check_hit_with_accelerometer_and_change(acc_data, time, parameters, actions['hit'])
    if not actions['swing']:
        actions['swing'] = events.check_swing(gyro_data, time, parameters)
    if not actions['stab']:
        actions['stab'] = events.check_stab(acc_data, gyro_data, time, parameters)
    parameters['w_prev'] = w_curr
    return actions


def main():
    acc_data = list()
    gyro_data = list()
    parameters = {"w_prev": 0, 'a_high': 0, 'w_rising': 0, 'w_low': 0, 'a_start': -1, 'w_start': -1, 'hit_start': -1,
                  'stab_start': -1,
                  'w_low_start': -1, 'hit_starts': []}
    actions = {'spin': 0, 'swing': 0, 'hit': 0, 'stab': 0}
    time = 0
    f = open("res_data.txt")
    for data in f:
        time += 1
        actions = get_new_states(acc_data, gyro_data, parameters, data, time, actions)


if __name__ == '__main__':
    main()
