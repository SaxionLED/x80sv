
import numpy as np
import matplotlib.pyplot as plt


t = np.linspace(0, 10, 1000)
dt = 10 / 1000

# parameters

def calculate_motion_profile(x0, xt, v0=0, vt=0, acc=1, v_max=1):
    """ Calculate a 2nd order motion profile from x1 to x2 given
        a certain constant acceleration and maximum velocity.

        Formula for position over time:
        s_t = s_0 + v_0 * t + 0.5 * a * t**2

        Velocity:
        v_t = v_0 + a * t

        Time it takes to reach v_max:
        v_max = a * t
        t = v_max / a
    """
    delta = xt - x0
    t_decel = v_max / acc
    t_accel = v_max / acc
    x_decel = 0.5 * acc * t_decel ** 2
    x_accel = 0.5 * acc * t_accel ** 2

    if delta < x_decel + x_accel:
        print('short version')
        t_cruise = 0
    else:
        x_cruise = delta - x_decel - x_accel
        t_cruise = x_cruise / v_max

    def f_acc(t):
        if t < t_accel:
            return acc
        elif t < t_accel + t_cruise:
            return 0
        elif t < t_accel + t_cruise + t_decel:
            return -acc
        else:
            return 0
    f_acc = np.vectorize(f_acc)
    a = f_acc(t)
    v = np.cumsum(a) * dt
    p = np.cumsum(v) * dt
    return a, v, p

a, v, p = calculate_motion_profile(0, 1.81)

# Time based planning
plt.subplot(3, 1, 1)
plt.title("Motion profile")
plt.ylabel('Position')
plt.grid(True)
plt.plot(t, p)

plt.subplot(3, 1, 2)
plt.ylabel('Velocity')
plt.grid(True)
plt.plot(t, v)

plt.subplot(3, 1, 3)
plt.ylabel('Acceleration')
plt.xlabel('Time [s]')
plt.grid(True)
plt.plot(t, a)

plt.show()

