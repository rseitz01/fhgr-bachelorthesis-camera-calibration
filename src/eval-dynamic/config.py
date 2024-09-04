import numpy as np

class Config:
# TODO make something better with below...
    DETECT_ALL_ON_FIRST_THRESHOLD               = 4 #0.4 
    DETECT_ALL_ON_FIRST_END_THRESOLD            = 4 #0.4 

    DETECT_ALL_ON_SIMILAR_ANGLE_MAX             = np.deg2rad(8) # np.deg2rad(6)
    DETECT_ALL_ON_SIMILAR_ANGLE_MIN             = np.deg2rad(1) # np.deg2rad(0.5)

    NUM_LEDS        = 16
    NUM_LEDS_EDGE   = 4

    ELLIPSE_MULT    = 30
    EVAL_MULT       = 30

    CALIBRATED_FREQUENCIES = {
            30: 30.005,
            40: 40.007,
            50: 50.009,
            56.250000: 56.259,
            59.70070048821906: 59.711,
            60: 60.010,
            60.2990834539315: 60.309,
            64.28571428571429: 64.296,
            75: 75.013,
            120: 120.02,
            284.9905003166561: 285.04,
            752.0053475935829: 752.16,
            4500: 4500.8,
            }

    CALIBRATED_MIN_MAX_STD = {
            30: [30.005, 30.005, 62.566e-6],
            40: [40.006, 40.007, 366.58e-6],
            50: [50.007, 50.009, 129.31e-6],
            56.250000: [56.258, 56.260, 368.78e-6],
            59.70070048821906: [59.709, 59.712, 483.37e-6],
            60: [60.009, 60.012, 425.85e-6],
            60.2990834539315: [60.309, 60.310, 498.19e-6],
            64.28571428571429: [64.296, 64.297, 478.72e-6],
            75: [75.012, 75.013, 483.33e-6],
            120: [120.02, 120.02, 1.1208e-3],
            284.9905003166561: [285.03, 285.05, 3.2476e-3],
            752.0053475935829: [752.08, 752.18, 20.029e-3],
            4500: [4499.6, 4501.7, 370.78e-3],
            }

    CALIBRATED_PWM = {
            0: [222e-6, 222e-6],
            1: [222e-6, 208e-6],
            2: [222e-6, 194e-6],
            3: [222e-6, 180e-6],
            4: [222e-6, 166e-6],
            5: [222e-6, 152e-6],
            6: [222e-6, 138e-6],
            7: [222e-6, 125e-6],
            8: [222e-6, 111e-6],
            9: [222e-6, 96.8e-6],
            10: [222e-6, 82.8e-6],
            11: [222e-6, 69.2e-6],
            12: [222e-6, 55.2e-6],
            13: [222e-6, 41.2e-6],
            14: [222e-6, 27.6e-6],
            15: [222e-6, 13.6e-6],
            16: [222e-6, 0e-6],
            }


    LED_PATTERN_NUMBER  = 0x842F
    LED_PATTERN_ORDER   = np.array([c == '1' for c in np.binary_repr(LED_PATTERN_NUMBER)]).reshape((NUM_LEDS_EDGE, NUM_LEDS_EDGE))
    LED_COLORS = ['red', 'chocolate', 'gold', 'olive',
                  'lawngreen', 'forestgreen', 'aquamarine', 'teal',
                  'deepskyblue', 'steelblue', 'blue', 'slateblue',
                  'blueviolet', 'violet', 'magenta', 'hotpink']

    # TODO: assert LED_COLORS len == NUM_LEDS...

