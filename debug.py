#import serial
from collections import deque
from math import sqrt

SWING_HIGH_W = 1000000  # threshold for angular acceleration for swing detect in CU(conditional unit)
SWING_LOW_W = 200000
STAB_LOW_W = 1000000  # low threshold for angular velocity for stab detect in CU
STAB_HIGH_A = 20000000  # threshold for acceleration for stab detect in CU
# HIT_HIGH_A = 0
HIT_HIGH_A = 200000000  # threshols for acceleration for hit detect in CU
SWING_HIGH_A = 21000000
SWING_TIME = 10  # number of measurements to detect swing
HIT_TIME = 10  # number of measurements to detect hit using acceleration
STAB_TIME = 5  # number of measurements to detect stab
HIT_PAUSE = 50  # minimal time pause between different hits
SWING_LOW_A = 17000000


def update_acc_data(parameters: dict, actions: dict, a_curr: int, time: int):
    """
    Function updates acceleration parameters: start ans state of acceleration rising
    :param parameters: dict with parameters
    :param a_curr: current acceleration data
    :param actions: list of actions states
    :param time time count
    :return:
    """
    if a_curr >= HIT_HIGH_A:
        parameters['a_start'] = time
    if a_curr >= SWING_HIGH_A and parameters['a_swing'] == 0:
        parameters['a_swing_start'] = time
        parameters['a_swing'] = 1
    if a_curr >= STAB_HIGH_A and parameters['a_stab'] == 0:
        parameters['a_stab'] = 1
        parameters['a_stab_start'] = time
    if a_curr < STAB_HIGH_A:
        parameters['a_stab'] = 0
        actions['stab'] = 0
    if a_curr < SWING_LOW_A:
        parameters['a_swing'] = 0


def update_gyro_data(parameters: dict, actions: dict, w_curr: int, time: int):
    """
    this function updates parameters that depend of gyroscope
    :param parameters: dict with parameters
    :param actions: dict with states
    :param w_curr: current angular velocity
    :param time time counter
    :return:
    """
    if parameters['w_prev'] < w_curr and parameters['w_rising'] == 0:
        parameters['w_rising'] = 1
        parameters['w_start'] = time
    """if parameters['w_prev'] > w_curr:
        parameters['w_rising'] = 0
        actions['swing'] = 0"""
    if w_curr < STAB_LOW_W and parameters['w_low'] == 0:
        parameters['w_low_start'] = time
        parameters['w_low'] = 1
    if w_curr > STAB_LOW_W:
        parameters['w_low'] = 0
        actions['stab'] = 0
    if w_curr < SWING_LOW_W:
        parameters['w_rising'] = 0


def check_hit_with_accelerometer_and_change(acc_data: deque, time: int, parameters: dict, hit: int) -> bool:
    """
    this function detects hits using high acceleration above HIT_HIGH_A
    :param acc_data: 10 last accelerometer measurements
    :param time: current time counter
    :param parameters:  parameters of system
    :param hit: state of hit action
    :return:
    """
    if (time - parameters['a_start']) < HIT_TIME:
        """if hit == 0 and (not parameters['hit_starts'] or time - parameters['hit_starts'][-1] > HIT_PAUSE):
            print('HIT! at %i' % time)
            if not time in parameters['hit_starts']:
                parameters['hit_starts'].append(time)
            return 1"""
        change = 0
        for i in range(min(HIT_TIME - 1, len(acc_data) - 1)):
            mul = sum([acc_data[0][j] * acc_data[i + 1][j] for j in range(3)])
            if mul < 0:
                change += 1
            if change > 0 and hit == 0 and (
                        not parameters['hit_starts'] or time - parameters['hit_starts'][-1] > HIT_PAUSE):
                print('HIT! at %i' % time)
                if time not in parameters['hit_starts']:
                    parameters['hit_starts'].append(time)
                return True
        if change == 0:
            return False
    return False


def check_hit_with_change(acc_data: deque, time: int, parameters, hit) -> bool:
    """
    Function detects if accelerometer data in acc_data contains hit
    hit as detected as more than one change of acceleration orientation through 10 measurements (100 ms)
    change of acceleration orientation is detected as scalar multiplication of acc vectors < 0
    :param acc_data: data with accelerometer measures
    :param time: current time counter
    :param parameters: list of current parameters
    :param hit is True if hit started
    :return: 1 if hit else 0
    """
    change = 0
    for i in range(len(acc_data) - 2):
        mul = sum([acc_data[i][j] * acc_data[i + 2][j] for j in range(3)])
        if mul < 0:
            change += 1
        if change > 0 and hit == 0:
            print('HIT! at %i' % time)
            if time not in parameters['hit_starts']:
                parameters['hit_starts'].append(time)
            return True
    if change == 0:
        return False


def check_new_swing(gyro_data, time, parameters, swing) -> bool:
    """
    function detects swing. Swing is detected if angular velocity rises during last 10 measurements
    and angular acceleration     for this time is more then SWING_HIGH_W threshold
    :param gyro_data:  queue with last ten gyro measurements
    :param time: current time counter
    :param parameters: dict with parameteres
    :return: 1 if swing else 0
    """
    if not swing:
        div = sum(
            [(gyro_data[SWING_TIME - 1][i] - gyro_data[0][i]) * (gyro_data[SWING_TIME - 1][i] - gyro_data[0][i]) for i
             in
             [1, 2]])
        if (parameters['w_rising'] and (time - parameters['w_start']) > SWING_TIME and div > SWING_HIGH_W) or (
            parameters['a_swing'] and (time - parameters['a_swing_start'] > SWING_TIME)):
            print('SWING started at %i' % time)
            if not time in parameters['swing_starts']:
                parameters['swing_starts'].append(time)
            return True
        return False
    if parameters['w_rising'] == 0:
        print('SWING ended at %i' % time)
        return False
    return True


def check_swing(gyro_data, time, parameters) -> bool:
    """
    function detects swing. Swing is detected if angular velocity rises during last 10 measurements
    and angular acceleration     for this time is more then SWING_HIGH_W threshold
    :param gyro_data:  queue with last ten gyro measurements
    :param time: current time counter
    :param parameters: dict with parameteres
    :return: 1 if swing else 0
    """
    if parameters['w_rising'] and (time - parameters['w_start']) > SWING_TIME:
        # div = sum([(gyro_data[SWING_TIME-1][i] - gyro_data[0][i]) * (gyro_data[SWING_TIME-1][i] - gyro_data[0][i]) for i in [1, 2]])
        div = sqrt(gyro_data[SWING_TIME - 1][1] * gyro_data[SWING_TIME - 1][1] + gyro_data[SWING_TIME - 1][2] *
                   gyro_data[SWING_TIME - 1][2])
        div -= sqrt(gyro_data[0][1] * gyro_data[0][1] + gyro_data[0][2] * gyro_data[0][2])
        if div / SWING_TIME > SWING_HIGH_W:
            print('SWING started at %i' % parameters['w_start'])
            if not parameters['w_start'] in parameters['swing_starts']:
                parameters['swing_starts'].append(parameters['w_start'])
            return True
    return False


def check_stab(acc_data, gyro_data, time, parameters) -> bool:
    """
    function detects stab action
    stab is action with high acceleration and low angular velocity
    :param acc_data: 10 last accelerometer measurements
    :param gyro_data: 10 last gyroscope measurements
    :param time: current time counter
    :param parameters: parameters of system
    :return: 1 is stab else 0
    """
    if parameters['a_stab'] and (time - parameters['a_stab_start']) > STAB_TIME and parameters['w_low']:
        if (time - parameters['w_low_start']) > STAB_TIME:
            print('RUN!!! at %i' % parameters['a_start'])
            if not parameters['a_start'] in parameters['stab_starts']:
                parameters['stab_starts'].append(parameters['a_start'])
        return True
    return False


def get_new_states(acc_data: deque, gyro_data: deque, parameters: dict, data: str, time: int, actions: dict) -> dict:
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
    update_acc_data(parameters, actions, a_curr, time)
    update_gyro_data(parameters, actions, w_curr, time)
    actions['hit'] = check_hit_with_accelerometer_and_change(acc_data, time, parameters, actions['hit'])
    if time > 10:
        actions['swing'] = check_new_swing(gyro_data, time, parameters, actions['swing'])
    if not actions['stab']:
        actions['stab'] = check_stab(acc_data, gyro_data, time, parameters)
    parameters['w_prev'] = w_curr
    return actions


def main():
    acc_data = deque(maxlen=10)
    gyro_data = deque(maxlen=10)
    parameters = {"w_prev": 0, 'a_high': 0, 'w_rising': 0, 'w_low': 0, 'a_start': -1, 'w_start': -1, 'hit_start': -1,
                  'stab_start': -1, 'a_swing': 0,
                  'a_stab_start': -1, 'a_stab': 0, 'w_low_start': -1, 'swing_starts': [], 'hit_starts': [],
                  'stab_starts': []}
    actions = {'spin': 0, 'swing': 0, 'hit': 0, 'stab': 0}
    time = 0
    f = open("res_data1.txt")
    for data in f:
        time += 1
        actions = get_new_states(acc_data, gyro_data, parameters, data, time, actions)
    print("Swing starts: %s" % " ".join(list(map(str, parameters['swing_starts']))))
    print("Number of swings %s" % len(parameters['swing_starts']))
    print("Hit starts: %s" % " ".join(list(map(str, parameters['hit_starts']))))
    print("Number of hits %s" % len(parameters['hit_starts']))
    print("Stab starts: %s" % " ".join(list(map(str, parameters['stab_starts']))))
    print("Number of stabs %s" % len(parameters['stab_starts']))


if __name__ == '__main__':
    main()
