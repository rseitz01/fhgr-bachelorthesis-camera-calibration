import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from config import Config

import matplotlib as mpl
mpl.rcParams['axes.formatter.useoffset'] = False

if __name__ == '__main__':


    x = []
    y = []
    y_mms = []
    for v in sorted(Config.CALIBRATED_FREQUENCIES):
        x.append(v)
        y.append(Config.CALIBRATED_FREQUENCIES[v])
        y_mms.append(np.array(Config.CALIBRATED_MIN_MAX_STD[v]))
    x = np.array(x)
    y = np.array(y)
    y_mms = np.array(y_mms)
    y_d = (y_mms[:,1] - y_mms[:,0]) / 2
    #print(x,y,y_mms)

    fig, ax1 = plt.subplots()
    ### plt.tight_layout()
    ### ax1.plot(x, (y-x)*1000, label='mHz')

    ### ax1.fill_between(x, 1000*(y-x+y_mms[:,2]), 1000*(y-x-y_mms[:,2]), alpha=0.3)

    ax1.set_xlabel('Target Frequency [Hz]')
    ax1.set_ylabel('Error [‰]')
    ax1.set_title('Target vs. Measured Frequency (room Temperature)')
    ### ax1.set_yscale('log')
    ax1.set_xscale('log')

    x_fmt = [
            30, 40, 50,
            #56.250000, 
            60,
            #64.28571428571429,
            75, 120,
            284.9905003166561,
            752.0053475935829,
            4500,
            ]
    #for i in range(len(x_fmt)):
    #    print(x_fmt[i], Config.CALIBRATED_FREQUENCIES[x_fmt[i]]-x_fmt[i])

    y_fmt = np.array([Config.CALIBRATED_FREQUENCIES[x]-x for x in x_fmt])
    ax1.set_xticks(x_fmt)
    ### ax1.set_yticks(1000*y_fmt)
    ax1.get_xaxis().set_major_formatter(mpl.ticker.ScalarFormatter())
    ### ax1.get_yaxis().set_major_formatter(mpl.ticker.ScalarFormatter())

    ### ax1.legend(loc='upper left')
    ### ax1.grid()


    #fig, ax1 = plt.subplots()

    y_ratio = y / x - 1

    y_ratio2 = y_mms[:,2] / y

    ### ax2 = ax1.twinx()
    ### ax2.fill_between(x, 1000*y_ratio, 1000*y_ratio2, color='orange', label='Error', alpha=0.3)
    ### ax2.fill_between(x, 1000*y_ratio2, color='coral', label='σ', alpha=0.6)
    ax1.fill_between(x, 1000*y_ratio, 1000*y_ratio2, color='orange', label='Error', alpha=0.3)
    ax1.fill_between(x, 1000*y_ratio2, color='coral', label='σ', alpha=0.6)

    y_fmt = np.array([Config.CALIBRATED_FREQUENCIES[x]-x for x in x_fmt])

    ax1.grid()
    ax1.legend(loc='upper right')

    #ax1.set_zorder(2)
    #ax2.set_zorder(1)

    #plt.legend()



    plt.show()

