
"""

    Create some plots of the speed issue.
    The issue is that the speed is calculated using derivative,
    this introduces a lot of noise.

    The issue can be resolved using an alpha-beta filter, which 
    is essentially an Luenberger observer with the following parameters:

    A = [ 1  dt
          0  1 ]
    C = [ 1  0 ]
    B = [ 0  0
          0  0 ]
    u = [ 0
          0 ]

    Alpha beta filter:
    
    Update state:
    x_hat_n = x_hat_n_1 = dt * v_hat_n_1

    Innovation signal:
    r_n = x_n - x_hat_n

    Update state estimates:
    x_hat_n = x_hat_n + alpha * r_n
    v_hat_n = v_hat_n + (beta / dt) * r_n

    [1] http://en.wikipedia.org/wiki/Alpha_beta_filter

"""

import pandas as pd
import numpy as np
from pandas.io.parsers import read_csv
from matplotlib import pyplot as plt
import ipdb

class AlphaBeta:
    """ Implements an alpha beta filter """
    def __init__(self, alpha=1.5, beta=0.5):
        self.alpha = alpha
        self.beta = beta
        self.x_est = 0
        self.v_est = 0
        self.dt = 0.1

        # Check parameters:
        assert 0 < alpha and alpha < 1
        assert 0 < beta and beta < 2
        assert 0 < 4 - 2*alpha - beta

    def add_sample(self, x):
        # Update estimations:
        self.x_est += self.dt * self.v_est

        # Innovation:
        r = x - self.x_est

        # Correct estimates, by using the innovation:
        self.x_est += self.alpha * r
        self.v_est += (self.beta / self.dt) * r
        return self.x_est, self.v_est

    def filter_data(self, data):
        """ Filter single dimensional array of data into two arrays containing
            position and velocity """
        x = []
        v = []

        # print(data)
        for rv in data:
            x_, v_ = self.add_sample(rv)
            x.append(x_)
            v.append(v_)
        return x, v


# Load data from csv file:
pos = read_csv('pos.csv', index_col=0)
pos.index = pd.to_datetime(pos.index)
vel = read_csv('vel.csv', index_col=0)
vel.index = pd.to_datetime(vel.index)

# Apply alpha beta filter:
alpha_beta = AlphaBeta(alpha=0.5, beta=0.1)
p_est, v_est = alpha_beta.filter_data(pos['field.right'].values)

# Create new and old arrays:
vel_new = np.array(v_est)
vel_old = vel['field.right'].values

# Plot the data:
plt.figure()
plt.hold(True)
plt.plot(vel_new)
plt.plot(vel_old)
#vel.plot()
#pos.plot()

# End:
#ipdb.set_trace()
plt.show()

