from collections import namedtuple
from math import cos, sin, sqrt, pi

from pyquaternion import Quaternion

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


def slow_quatern_from_data(prev: Quaternion, data: str, delay: float) -> Quaternion:
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
    return prev*Quaternion(w, x, y, z)


def madgwick_filtered(prev_q: Quaternion, data: str, delay: float, beta: float = 0.1) -> Quaternion:
    """
    expected that data contains radians
    :param data:
    :param delay:
    :return:
    """
    (ax, ay, az), (dgx, dgy, dgz) = data_split(data)
    gx, gy, gz = gyro_to_rad(dgx, dgy, dgz)
    w, x, y, z = prev_q.elements
    # Auxiliary variables to avoid repeated arithmetic
    _2w = 2 * w
    _2x = 2 * x
    _2y = 2 * y
    _2z = 2 * z
    _4w = 4 * w
    _4x = 4 * x
    _4y = 4 * y
    _8x = 8 * x
    _8y = 8 * y
    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z
    # // Normalise accelerometer measurement
    acc_norm = sqrt(ax * ax + ay * ay + az * az)
    if (acc_norm == 0):
        return prev_q
    acc_norm = 1 / acc_norm
    ax = ax * acc_norm
    ay = ay * acc_norm
    az = az * acc_norm

    # // Gradient decent algorithm corrective step
    s1 = _4w * yy + _2y * ax + _4w * xx - _2x * ay
    s2 = _4x * zz - _2z * ax + 4 * ww * x - _2w * ay - _4x + _8x * xx + _8x * yy + _4x * az
    s3 = 4 * ww * y + _2w * ax + _4y * zz - _2z * ay - _4y + _8y * xx + _8y * yy + _4y * az
    s4 = 4 * xx * z - _2x * ax + 4 * yy * z - _2y * ay
    filter_norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)  # // normalise step magnitude
    s1 = s1 * filter_norm
    s2 = s2 * filter_norm
    s3 = s3 * filter_norm
    s4 = s4 * filter_norm

    # // Compute rate of change of quaternion
    qDot1 = 0.5 * (-x * gx - y * gy - z * gz) - beta * s1
    qDot2 = 0.5 * (w * gx + y * gz - z * gy) - beta * s2
    qDot3 = 0.5 * (w * gy - x * gz + z * gx) - beta * s3
    qDot4 = 0.5 * (w * gz + x * gy - y * gx) - beta * s4
    # // Integrate to yield quaternion
    w += qDot1 * delay
    x += qDot2 * delay
    y += qDot3 * delay
    z += qDot4 * delay
    # // normalise quaternion
    q_norm = 1.0 / sqrt(w * w + x * x + y * y + z * z)
    return Quaternion(w * q_norm, x * q_norm, y * q_norm, z * q_norm)


def raw_quatern_from_data(prev_q: Quaternion, data: str, delay: float) -> Quaternion:
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


def gyro_to_rad(gx: int, gy: int, gz: int) -> (float, float, float):
    """
    convert gyro data to radians, by Camill
    Could be wrong
    """
    # TODO: request assistance
    g_s = 2000.0 / 32768 / 180 * pi  # GYRO_SCALE
    return gx * g_s, gy * g_s, gz * g_s