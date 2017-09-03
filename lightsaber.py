import serial
from collections import deque

SWING_HIGH_W =  150000     #threshold for angular acceleration for swing detect in CU(conditional unit)
STAB_LOW_W = 10000        #low threshold for angular velocity for stab detect in CU
STAB_HIGH_A = 10000000    #threshold for acceleration for stab detect in CU
HIT_HIGH_A = 16000000      #threshols for acceleration for hit detect in CU
SWING_TIME = 10           #number of measurements to detect swing
HIT_TIME = 5              #number of measurements to detect hit using acceleration
STAB_TIME = 5             #number of measurements to detect stab

def update_acc_data(parameters:dict, actions: dict,  a_curr: int, time: int):
    """
    Function updates acceleration parameters: start ans state of acceleration rising
    :param parameters: dict with parameters
    :param a_curr: current acceleration data
    :param action: list of actions states
    :return:
    """
    if parameters['a_prev'] < a_curr and parameters['a_rising'] == 0:
        parameters['a_rising'] = 1
        parameters['a_start'] = time
    if parameters['a_prev'] > a_curr:
        parameters['a_rising'] = 0
        actions['stab'] = 0

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
    if parameters['a_rising'] and (time - parameters['a_start']) > HIT_TIME:
        div = sum([acc_data[0][i] * acc_data[HIT_TIME-1][i] for i in range(3)])
        if div > HIT_HIGH_A:
            return 1
    return 0


def check_hit_with_change(acc_data: deque, time: int, parameters, hit) ->bool:
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
    for i in range(len(acc_data)-2):
        mul = sum([acc_data[i][j]*acc_data[i+2][j] for j in range(3)])
        if mul<0:
            change+=1
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
        div = sum([(gyro_data[SWING_TIME-1][i] - gyro_data[0][i]) * (gyro_data[SWING_TIME-1][i] - gyro_data[0][i]) for i in range(3)])
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
    if div > STAB_HIGH_A and parameters['w_low'] and time-parameters['w_low_start'] > STAB_TIME:
        return 1
    return 0

def get_new_states(acc_data: deque, gyro_data: deque, parameters: dict, data:str, time: int, actions: dict) -> dict:
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
    a_curr = sum([accel[i]*accel[i] for i in range(3)])
    w_curr = sum([gyro[i]*gyro[i] for i in range(3)])
    update_acc_data(parameters, actions, a_curr, time)
    update_gyro_data(parameters, actions, w_curr, time)
    actions['hit'] = check_hit_with_change(acc_data, time, parameters, actions['hit'])
    if not actions['swing']:
        actions['swing'] = check_swing(gyro_data, time, parameters)
    if not actions['stab']:
        actions['stab'] = check_stab(acc_data, gyro_data, time, parameters)
    parameters['a_prev'] = a_curr
    parameters['w_prev'] = w_curr
    return actions

def main():

    acc_data = deque(maxlen=10)
    gyro_data = deque(maxlen=10)
    parameters = {"a_prev":0,  "w_prev":0, 'a_rising':0, 'w_rising':0,  'w_low':0, 'a_start':-1,  'w_start':-1, 'hit_start': -1, 'stab_start':-1,
                  'w_low_start':-1, 'swing_starts':[], 'hit_starts':[], 'stab_starts':[]}
    actions = {'spin':0, 'swing':0, 'hit':0, 'stab':0}
    time = 0
    f = open("res_data1.txt")
    for data in f:
        time+=1
        actions = get_new_states(acc_data, gyro_data, parameters, data, time, actions)

if __name__ == '__main__':
    main()
