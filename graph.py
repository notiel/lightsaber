from matplotlib import pyplot as plot
from mpl_toolkits.mplot3d import Axes3D


def plot_swings(detected_events,gyro_data):
    plot.figure()
    ox = back_gyro(gyro_data)
    swing_stats = [y['swing'] for y in detected_events]
    swing_dots = [2500 if state else 0 for state in swing_stats]
    plot.plot(ox, swing_dots, 'black', label='swingsers', )
    plot.xlabel('ms')
    plot.ylabel('acc_data')
    plot.legend()
    pass


def back_acc(acc_data):
    ox = [x for x in range(0, len(acc_data))]
    for label in ['acc_x', 'acc_y', 'acc_z']:
        index = ['acc_x', 'acc_y', 'acc_z'].index(label)
        dots = [y[index] for y in acc_data]
        plot.plot(ox, dots, label=label)
    return ox


def back_gyro(gyro_data):
    ox = [x for x in range(0, len(gyro_data))]
    for label in ['gyro_x', 'gyro_y', 'gyro_z']:
        index = ['gyro_x', 'gyro_y', 'gyro_z'].index(label)
        dots = [y[index] for y in gyro_data]
        plot.plot(ox, dots, label=label)
    return ox


def plot_quatern_wx(quatertn_data, gyro_data):
    plot.figure()
    ox = back_gyro(gyro_data)  # swings:
    q_w = [(y[0] * 10000) - 10000 for y in quatertn_data]
    plot.plot(ox, q_w, 'black', label='q_w')
    q_x = [y[1] * 10000 for y in quatertn_data]
    plot.plot(ox, q_x, 'cyan', label='q_x')
    plot.legend()
    return None


def plot_quatern_yz(quatertn_data, gyro_data):
    plot.figure()
    ox = back_gyro(gyro_data)  # swings:
    q_y = [y[2] * 10000 for y in quatertn_data]
    plot.plot(ox, q_y, 'black', label='q_y')
    q_z = [y[3] * 10000 for y in quatertn_data]
    plot.plot(ox, q_z, 'cyan', label='q_z')
    plot.legend()
    return None


def plot_vector3(listx, listy, listz):
    fig=plot.figure()
    ax=fig.add_subplot(111, projection='3d')
    ax.plot(xs=listx, ys=listy, zs=listz)
    pass


def plot_quaternion_evo(qlist):
    listw, listx, listy, listz = zip (*qlist)
    plot_vector3(listx, listy, listz)
    pass


def plot_vector_evo(vlist):
    listx, listy, listz = zip(*vlist)
    plot_vector3(listx, listy, listz)
    return None