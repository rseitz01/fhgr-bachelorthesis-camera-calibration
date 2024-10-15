import numpy as np
from config import Config
from scipy import interpolate

# calculate center of mass (circular)
def estimate_center_mass(arr):
    # take circular approach
    phi_delta = 2*np.pi/Config.NUM_LEDS
    phi = 0
    total = np.array([0.0, 0.0])
    for v in arr:
        total += np.array([(np.sin(phi) * v), (np.cos(phi) * v)])
        phi += phi_delta
    center = np.arctan2(total[0], total[1])
    return center

class PositionSequence:

    def __init__(self):
        self.__pwm_data = None
        self.__angles = {}
        self.__angles_noncorrected = {}
        self.__width_raw = {}
        self.__width_cor = {}

    # clear everything
    def clear(self):
        self.__init__()

    # calibrate pwm brightnesses
    def set_pwm_calibration(self, data):
        self.__pwm_data = np.array(data)
        #print("DATA",self.__pwm_data)
        assert(self.__pwm_data.shape[0] == Config.NUM_LEDS)
        self.__pwm_data = np.clip(self.__pwm_data, 0, 1)

        #xs = np.linspace(0, 1, Config.NUM_LEDS)
        assert(len(Config.CALIBRATED_PWM) == self.__pwm_data.shape[1])
        xs = np.array([(Config.CALIBRATED_PWM[x][0]-Config.CALIBRATED_PWM[x][1]) / Config.CALIBRATED_PWM[x][0] for x in range(len(Config.CALIBRATED_PWM))])

        self.__index_min_max = []
        for i, y in enumerate(self.__pwm_data):
            max_val = None
            for j, yy in enumerate(y):
                if max_val is None or yy > max_val:
                    max_val = yy
                if yy <= max_val:
                    max_val += 1e-9
                    self.__pwm_data[i][j] = max_val
                if np.isclose(yy, 1.0) and len(self.__index_min_max) <= i:
                    self.__index_min_max.append(j)
                if j == 0:
                    self.__pwm_data[i][j] = 0
                if j+1 == len(y):
                    max_val = max(max_val, 1)
                    max_val += 1e-9
                    self.__pwm_data[i][j] = max_val
            if len(self.__index_min_max) <= i:
                self.__index_min_max.append(Config.NUM_LEDS - 1)

        assert(len(self.__index_min_max) == Config.NUM_LEDS)

        self.__corrections = [interpolate.interpolate.interp1d(y, xs, axis=0) for y in self.__pwm_data]
        self.__index_min_max = np.array(self.__index_min_max)

    # get reverse pwm interpolation after calibrating
    def get_pwm_interpolation(self, data):
        ys = np.clip(data, 0, 1)
        xs = np.array([self.__corrections[i](ys) for i in range(Config.NUM_LEDS)])
        return xs

    # get widths (exposure) from values
    def width_from_values(self, values, index):
        if self.__pwm_data is None:
            raise Exception('Not (yet) supported to not have PWM calibration data when getting angles...')
        if index in self.__width_cor:
            return self.__width_cor[index], self.__width_raw[index]

        values = np.clip(values, 0, 1)
        values_cor = np.clip(np.array([self.__corrections[i](values[i]) for i in range(Config.NUM_LEDS)]), 0, 1)
        values_close_one = np.where(np.isclose(values_cor, 1))

        for i in values_close_one:
            values_cor[i] = self.__index_min_max[i] / (Config.NUM_LEDS - 1)

        width_cor = np.sum(values_cor) * 1 / np.max(values_cor)
        width_raw = np.sum(values) * 1 / np.max(values)
        self.__width_cor[index] = width_cor
        self.__width_raw[index] = width_raw
        return width_cor

    # get angles (position) from values
    def angle_from_values(self, values, index):
        if self.__pwm_data is None:
            raise Exception('Not (yet) supported to not have PWM calibration data when getting angles...')
        if index in self.__angles:
            return self.__angles[index], self.__angles_noncorrected[index]

        values = np.clip(values, 0, 1)
        values_corrected = np.clip(np.array([self.__corrections[i](values[i]) for i in range(Config.NUM_LEDS)]), 0, 1)
        values_close_one = np.where(np.isclose(values_corrected, 1))
        for i in values_close_one:
            values_corrected[i] = self.__index_min_max[i] / (Config.NUM_LEDS - 1)

        angle = estimate_center_mass(values_corrected)
        angle_noncorrected = estimate_center_mass(values)

        self.__angles[index] = angle
        self.__angles_noncorrected[index] = angle_noncorrected
        return angle

    # retrieve stored widths
    def widths_stored(self):
        if len(self.__width_raw):
            width_raw = np.array([self.__width_raw[i] for i in self.__width_raw])
            width_cor = np.array([self.__width_cor[i] for i in self.__width_cor])
            return width_cor, width_raw
        else:
            return None

    # retrieve stored angles
    def angles_stored(self):
        if len(self.__angles):
            angles = np.array([self.__angles[angle] for angle in self.__angles])
            angles_nonc = np.array([self.__angles_noncorrected[angle] for angle in self.__angles_noncorrected])
            return angles, angles_nonc
        else:
            return None



