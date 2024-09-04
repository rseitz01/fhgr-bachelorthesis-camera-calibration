from positions import configs
import cv2
from cv2.typing import MatLike
import numpy as np
import matplotlib.pyplot as plt
import scipy

NUM_LEDS = 16


def estimate_center_mass(arr: np.ndarray) -> float:
    # take circular approach - inspired from
    # https://www.sciencetopia.net/physics/center-gravity
    phi_delta = 2*np.pi/NUM_LEDS
    phi = 0
    total = np.array([0.0, 0.0])
    for v in arr:
        total += np.array([np.sin(phi) * v, np.cos(phi) * v])
        phi += phi_delta
    #center = np.array(total) / NUM_LEDS
    #center = total / NUM_LEDS / 2 / np.pi
    center = np.rad2deg(np.arctan2(total[0], total[1]))
    return center

def get_state_at_pos(frame: MatLike, position: np.ndarray, w: int) -> float:
    y = position[1] - w
    x = position[0] - w
    img = frame[y:y+w, x:x+w]
    return np.average(img)


def retrieve_values(frame: MatLike, positions: np.ndarray, w: int) -> np.ndarray:
    result = []
    for position in positions:
        result.append(get_state_at_pos(frame, position, w))
    return np.array(result)

def get_diff(cal_off: None | np.ndarray, cal_100: None | np.ndarray, values: np.ndarray) -> np.ndarray:
    assert(cal_off is not None)
    diff: np.ndarray
    if cal_100 is not None:
        diff = (values - cal_off) / (cal_100 - cal_off)
        diff = np.clip(diff, -1, 1)
        diff *= 20 # TODO this is ugly af
    else:
        diff = values - cal_off
    #print("DIFF",diff)
    return diff



def estimate(filename: str):
    capture = cv2.VideoCapture(filename)
    frame_number = -1
    w = 5
    cal_off = None
    cal_100 = None
    maximum = None
    prescaler = 0 # determined from recording
    autoreload = 0 # determined from recording
    recurring_center = False
    center_prev = None
    shifts = []
    config = configs[filename]

    while 1:
        _, frame = capture.read()
        if frame is None: break
        frame_number += 1
        print(frame_number, end='\r')
        # process
        values = retrieve_values(frame, config['positions'], w)
        if frame_number == config['t_off']: # all LEDs off
            cal_off = values
            print("OFF:", cal_off)
        elif frame_number == config['t_100']: # all LEDs on
            cal_100 = values
            print("100:", cal_100)
        elif frame_number == config['t_psc']: # prescaler
            # convert to binary and then to number
            diff = get_diff(cal_off, cal_100, values)
            threshold = 10
            binary = np.where(diff > threshold, 1, 0)
            packed = np.packbits(binary)
            prescaler = int.from_bytes(packed.tobytes(), byteorder='big')
            print(f"PSC = {prescaler}")
        elif frame_number == config['t_arr']: # auto reload
            # convert to binary and then to number
            diff = get_diff(cal_off, cal_100, values)
            threshold = 10
            binary = np.where(diff > threshold, 1, 0)
            packed = np.packbits(binary)
            autoreload = int.from_bytes(packed.tobytes(), byteorder='big')
            print(f"ARR = {autoreload}")
        elif frame_number >= config['t_run']:
            #diff = values - cal_off
            diff = get_diff(cal_off, cal_100, values)
            #print("DIFF=",diff)
            center = estimate_center_mass(diff)
            if recurring_center and center_prev is not None:
                shift = 0.0
                if center_prev > 0 and center > 0 or center_prev < 0 and center < 0:
                    shift = center_prev - center
                elif center_prev < 0 and center > 0 or center_prev > 0 and center < 0:
                    shift = center_prev + center
                shifts.append(shift)
            else:
                recurring_center = True
            center_prev = center

        #cv2.imshow("frame", frame);
        #cv2.waitKey(1)


    N = 13
    arr = shifts
    arr = scipy.signal.medfilt(arr, 2*N+1)
    if 0:
        filtered = np.deg2rad(np.array(arr))
        filtered = filtered / (2*np.pi * NUM_LEDS**2 * (8000000/(prescaler+1)/autoreload/2/16 ))
        filtered *= 1000
        arr = filtered
    print(f"Average shift: {np.average(arr):.4f} ms")
    print(f"Total shift: {np.sum(arr):.4f} ms")
    #plt.plot(arr, '.')
    plt.plot(arr)
    plt.xlabel('frame')
    plt.ylabel('shift [ms]')
    plt.grid()
    plt.show()



if __name__ == '__main__':
    #estimate("./data/ledrunner_psc8-arr461.mp4")
    #estimate("./data/ledrunner_psc8-arr462.mp4")
    #estimate("./data/ledrunner_psc8-arr463.mp4")
    #estimate("./data/ledrunner_psc8-arr461.mp4")
    #estimate("./data/ledrunner_psc8-arr462.mp4")
    estimate("./data/ledrunner_psc8-arr462-500mb.mp4")
    #estimate("./data/ledrunner_psc8-arr463.mp4")

# https://stackoverflow.com/questions/58595510/loop-through-all-the-frames-in-the-video-python
# https://numpy.org/doc/stable/reference/generated/numpy.average.html
# https://stackoverflow.com/questions/66171969/compute-center-of-mass-in-numpy
