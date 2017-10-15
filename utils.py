A_OFFSET = [169, -52, 120]
G_OFFSET = [45, 95, -26]
G_SCALE = 2000.0/32768/180*3.1415
A_SCALE = 9.81/4096

def data_split(data: str) -> (list, list):
    """
    splits data flow from specific strings into two lists
    >>> data_split ("3909 304 -1591; 82 11 -39")
    ([3909, 304, -1591], [82, 11, -39])
    """
    data = data.split(';')
    accel = list(map(float, data[0].split()))
    for i in range(0,3):
        accel[i] = (accel[i]-A_OFFSET[i])*A_SCALE
    gyro = list(map(float, data[1].split()))
    for i in range(0,3):
        gyro[i] = (gyro[i]-G_OFFSET[i])*G_SCALE
    return accel, gyro