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