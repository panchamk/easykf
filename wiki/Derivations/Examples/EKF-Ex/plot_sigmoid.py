#!/usr/bin/env python

'''
Plotting a sigmoid
'''

import numpy as np
import matplotlib.pyplot as plt

def sigmoid(x, A, x0, b, k):
    return A / (1.0 + np.exp(k*(x - x0)))+b

# --- main
if __name__ == '__main__':
    fig = plt.figure(figsize=(10,10))
    ax1 = fig.add_subplot(111)
    #ax1.set_aspect('equal')

    x = np.arange(-10,10,0.1)
    sig = sigmoid(x, 2.0, -1.0, -1.0, -1.0)
    ax1.plot(x, sig,linewidth=2)

    plt.show()
