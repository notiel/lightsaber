from collections import deque

SWING_HIGH_W = 150000  # threshold for angular acceleration for swing detect in CU(conditional unit)
STAB_LOW_W = 10000  # low threshold for angular velocity for stab detect in CU
STAB_HIGH_A = 10000000  # threshold for acceleration for stab detect in CU
HIT_HIGH_A = 85000000  # threshols for acceleration for hit detect in CU
SWING_TIME = 10  # number of measurements to detect swing
HIT_TIME = 10  # number of measurements to detect hit using acceleration
STAB_TIME = 5  # number of measurements to detect stab
HIT_PAUSE = 50  # minimal time pause between different hits


def update_acc_data(parameters: dict, actions: dict, a_curr: int, time: int):
    """
    Function updates acceleration parameters: start ans state of acceleration rising
    :param parameters: dict with parameters
    :param a_curr: current acceleration data
    :param action: list of actions states
    :return:
    """
    if a_curr >= HIT_HIGH_A:
        parameters['a_start'] = time


def update_gyro_data(parameters: dict, actions: dict, w_curr: int, time: int):
    """
    this function updates parameters that depend of gyroscope
    :param parameters: dict with parameters
    :param actions: dict with states
    :param w_curr: current angular velocity
    :param stab_low_w - level of low angular velocity
    :return:
    """
    if parameters['w_prev'] < w_curr and parameters['w_rising'] == 0:
        parameters['w_rising'] = 1
        parameters['w_start'] = time
    if parameters['w_prev'] > w_curr:
        parameters['w_rising'] = 0
        actions['swing'] = 0
    if w_curr < STAB_LOW_W and parameters['w_low'] == 0:
        parameters['w_low_start'] = time
        parameters['w_low'] = 1
    if w_curr > STAB_LOW_W:
        parameters['w_low'] = 0
        actions['stab'] = 0


def check_hit_with_accelerometer_and_change(acc_data: deque, time: int, parameters: dict, hit: int) -> bool:
    """
    this function detects hits using high acceleration above ACC_HIT_LEVEL
    :param acc_data: 10 last accelerometer measurements
    :param time: current time counter
    :param parameters:  parameters of system
    :param hit: state of hit action
    :return:
    """
    if (time - parameters['a_start']) < HIT_TIME:
        change = 0
        for i in range(min(HIT_TIME - 1, len(acc_data) - 1)):
            mul = sum([acc_data[0][j] * acc_data[i + 1][j] for j in range(3)])
            if mul < 0:
                change += 1
            if change > 0 and hit == 0 and (
                        not parameters['hit_starts'] or time - parameters['hit_starts'][-1] > HIT_PAUSE):
                print('HIT! at %i' % time)
                if not time in parameters['hit_starts']:
                    parameters['hit_starts'].append(time)
                return 1
        if change == 0:
            return 0
    return 0


def check_hit_with_change(acc_data: deque, time: int, parameters, hit) -> bool:
    """
    Function detects if accelerometer data in acc_data contains hit
    hit as detected as more than one change of acceleration orientation through 10 measurements (100 ms)
    change of acceleration orientation is detected as scalar multiplication of acc vectors < 0
    :param acc_data: data with accelerometer measures
    :param time: current time counter
    :param parameters: list of current parameters
    :return: 1 if hit else 0
    """
    change = 0
    for i in range(len(acc_data) - 2):
        mul = sum([acc_data[i][j] * acc_data[i + 2][j] for j in range(3)])
        if mul < 0:
            change += 1
        if change > 0 and hit == 0:
            return 1
    if change == 0:
        return 0


def check_swing(gyro_data, time, parameters) -> bool:
    """
    function detects swing. Swing is detected if angular velocity rises during last 10 measurements and angular acceleration
    for this time is more then SWING_HIGH_W threshold
    :param gyro_data:  queue with last ten gyro measurements
    :param time: current time counter
    :param parameters: dict with parameteres
    :return: 1 if swing else 0
    """
    if parameters['w_rising'] and (time - parameters['w_start']) > SWING_TIME:
        div = sum(
            [(gyro_data[SWING_TIME - 1][i] - gyro_data[0][i]) * (gyro_data[SWING_TIME - 1][i] - gyro_data[0][i]) for i
             in range(3)])
        if div / SWING_TIME > SWING_HIGH_W:
            return 1
    return 0


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
    div = sum([(acc_data[-1][i] - acc_data[0][i]) * (acc_data[-1][i] - acc_data[0][i]) for i in range(3)])
    if div > STAB_HIGH_A and parameters['w_low'] and time - parameters['w_low_start'] > STAB_TIME:
        return 1
    return 0