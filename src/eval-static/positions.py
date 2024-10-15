import numpy as np
positions_first = np.array([
        [465, 805],
        [701, 806],
        [932, 803],
        [1167,803],

        [462, 571],
        [699, 569],
        [935, 569],
        [1166,567],

        [467, 335],
        [699, 335],
        [932, 336],
        [1164,335],

        [465, 100],
        [699, 101],
        [933, 102],
        [1163,103],
    ])

positions_500mb = np.array([
        [472, 813],
        [650, 809],
        [827, 803],
        [1006,797],

        [467, 636],
        [644, 630],
        [822, 625],
        [1000,618],

        [464, 461],
        [639, 453],
        [816, 448],
        [994, 442],

        [459, 285],
        [636, 278],
        [812, 271],
        [988, 267],
    ])

configs = {
        './data/ledrunner_psc8-arr461.mp4': {
            'positions': positions_first,
            't_off': 0,
            't_psc': 100,
            't_arr': 333,
            't_run': 500,
            't_100': -1,
            },
        './data/ledrunner_psc8-arr462.mp4': {
            'positions': positions_first,
            't_off': 0,
            't_psc': 100,
            't_arr': 333,
            't_run': 500,
            't_100': -1,
            },
        './data/ledrunner_psc8-arr463.mp4': {
            'positions': positions_first,
            't_off': 0,
            't_psc': 100,
            't_arr': 333,
            't_run': 500,
            't_100': -1,
            },
        './data/ledrunner_psc8-arr462-500mb.mp4': {
            'positions': positions_500mb,
            't_off': 350,
            't_100': 520,
            't_psc': 600,
            't_arr': 660,
            't_run': 720
            }
        }


