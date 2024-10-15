import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from config import Config

import matplotlib as mpl
mpl.rcParams['axes.formatter.useoffset'] = False

if __name__ == '__main__':

    f = 4.5008e3
    steps = 16
    measured = Config.CALIBRATED_PWM

    for x in measured:
        p1 = x/(steps)*100
        p2 = (measured[x][0] - measured[x][1])*100/measured[x][0]
        if p1 == 0: continue
        #print(f'[{x}] {p1:.2f}% : {p2:.2f}% / {100*(p2/p1-1):.2f}% error')
        print(f'{p1:.2f} & {p2:.2f} & {100*(p2/p1-1):.2f} \\\\')


