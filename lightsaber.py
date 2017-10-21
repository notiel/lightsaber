# import serial
from collections import deque
from matplotlib import pyplot as plot

import events
from logging_logic import create_data_storage, calculate_and_collect, plot_collected
from utils import data_split

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
    if (time == 10):
        pass
    accel, gyro = data_split(data)
    acc_data.append(accel)
    gyro_data.append(gyro)
    a_curr = sum([accel[i] * accel[i] for i in range(3)])
    w_curr = sum([gyro[i] * gyro[i] for i in [1, 2]])
    events.update_acc_data(parameters, actions, a_curr, time)
    events.update_gyro_data(parameters, actions, w_curr, time)
    if time > 10:
        actions['swing'] = events.check_dynamic_swing(gyro_data, time, parameters, actions)
        #print("SWING = %s" % actions['swing'])
        actions['hit'] = events.check_hit_with_accelerometer_and_change(acc_data, time, parameters, actions['hit'])
        #if actions['swing']:
        #    actions['spin'] = events.check_spin(time, parameters, actions['spin'])

    #if not actions['stab']:
     #   actions['stab'] = events.check_stab(acc_data, gyro_data, time, parameters)
    parameters['w_prev'] = w_curr
    return actions

def main():
    acc_data = deque(maxlen=10)
    gyro_data = deque(maxlen=10)

    parameters = {"w_prev": 0, 'a_high': 0, 'w_rising': 0, 'w_low': 0, 'a_hit_start': -1, 'w_start': -1, 'hit_start': -1,
                  'stab_start': -1, 'a_swing': 0, 'swing_stop': 0, "w_swing": 0, 'w_swing_max': events.SWING_LOW_W, 'swing_num':0,
                  'a_stab_start': -1, 'a_stab': 0, 'w_low_start': -1, 'swing_starts': [], 'hit_starts': [],
                  'stab_starts': [], 'spin_starts':[]}
    actions = {'spin': 0, 'swing': 0, 'hit': 0, 'stab': 0}
    time = 0
    f = open("res/IMU-clash.txt")
    config = {
        'delay': 0.01,
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

    for data in f:
        time += 1
        if time == 210:
           print(time)

        actions = get_new_states(acc_data, gyro_data, parameters, data, time, actions)
        #calculate_and_collect(data, config, log_storage, actions=actions)

   # plot_collected(config, log_storage)
    #plot.show()


    print("Swing starts: %s" % " ".join(list(map(str, parameters['swing_starts']))))
    print("Number of swings %s" % len(parameters['swing_starts']))
    print("Hit starts: %s" % " ".join(list(map(str, parameters['hit_starts']))))
    print("Number of hits %s" % len(parameters['hit_starts']))
    print("Stab starts: %s" % " ".join(list(map(str, parameters['stab_starts']))))
    print("Number of stabs %s" % len(parameters['stab_starts']))
    print("Spin starts: %s" % " ".join(list(map(str, parameters['spin_starts']))))
    print("Number of spins %s" % len(parameters['spin_starts']))








if __name__ == '__main__':
    main()
