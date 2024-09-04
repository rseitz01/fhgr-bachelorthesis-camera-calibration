import cv2
import numpy as np
import itertools
import scipy
import matplotlib.pyplot as plt
from cv2.typing import MatLike
from scipy.spatial import ConvexHull

NUM_LEDS = 16


selected_points = []

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

def mouse_callback(event, x, y, flags, param):
    global selected_points
    if event==cv2.EVENT_LBUTTONDOWN:
        selected_points.append(np.array([x,y]))


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


def estimate(filename):
    global selected_points
    capture = cv2.VideoCapture(filename)
    if not capture.isOpened():
        print(f"Could not open '{filename}'")
        exit(1)

    show = None
    frame_number = -1
    frame_next = True

    frame_zero = None

    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('frame', mouse_callback)

    while 1:
        if frame_next:
            frame_next = False
            _, frame = capture.read()
            show = frame
            if frame is None: break
            frame_number += 1
            selected_points = []
            if frame_number == 0:
                frame_zero = frame
                continue

            if 0:
                assert(frame_zero is not None)
                diff = cv2.absdiff(frame_zero, frame)
                gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
                _, binary = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)

                contours, hier = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours_good_indices = []
                contours_good_centers = []
                for i in range(len(contours)):
                    epsilon = 0.05*cv2.arcLength(contours[i],True)
                    approx = cv2.approxPolyDP(contours[i],epsilon,True)
                    if len(approx) == 4:
                    #print(contour)
                        cv2.drawContours(diff, contours, i, (0,255,0), 1)
                        contours_good_indices.append(i)

                for i in contours_good_indices:
                    M = cv2.moments(contours[i])
                    if M['m00'] != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        contours_good_centers.append([cx, cy])
                        cv2.circle(diff, (cx, cy), 4, (0, 0, 255), -1)

                for i in range(len(contours_good_centers)):
                    for j in range(len(contours_good_centers)-1):
                        c1 = contours_good_centers[i]
                        c2 = contours_good_centers[j+1%len(contours_good_centers)]
                        cv2.line(diff, c1, c2, (255,0,0), 1)

            #image, contours, hier = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            #count = len(contours)
            #print("Count:",count)

        key = cv2.waitKey(1)
        if(key == ord('q')):
            exit(0)
        if(key == ord(' ')):
            frame_next = True

        if(show is not None):
            cv2.imshow("frame", show)

        if len(selected_points) == 4:
            break

    frame_number_all_one = frame_number
    locations_wrong_order = np.empty(NUM_LEDS)
    if len(selected_points) == 4:

        for p in selected_points:
            cv2.circle(show, p, 3, (0,0,255), -1)

        calculated_points = []

        x = selected_points[1] - selected_points[0]
        x_len = np.linalg.norm(x)
        x_hat = x / x_len
        p1 = np.round(selected_points[0] + x_hat * 1/3 * x_len).astype(int)
        p2 = np.round(selected_points[0] + x_hat * 2/3 * x_len).astype(int)
        calculated_points.append(p1)
        calculated_points.append(p2)

        x = selected_points[3] - selected_points[2]
        x_len = np.linalg.norm(x)
        x_hat = x / x_len
        p1 = np.round(selected_points[2] + x_hat * 1/3 * x_len).astype(int)
        p2 = np.round(selected_points[2] + x_hat * 2/3 * x_len).astype(int)
        calculated_points.append(p1)
        calculated_points.append(p2)

        x = selected_points[3] - selected_points[0]
        x_len = np.linalg.norm(x)
        x_hat = x / x_len
        p1 = np.round(selected_points[0] + x_hat * 1/3 * x_len).astype(int)
        p2 = np.round(selected_points[0] + x_hat * 2/3 * x_len).astype(int)
        calculated_points.append(p1)
        calculated_points.append(p2)

        x = selected_points[2] - selected_points[1]
        x_len = np.linalg.norm(x)
        x_hat = x / x_len
        p1 = np.round(selected_points[1] + x_hat * 1/3 * x_len).astype(int)
        p2 = np.round(selected_points[1] + x_hat * 2/3 * x_len).astype(int)
        calculated_points.append(p1)
        calculated_points.append(p2)

        x = calculated_points[3] - calculated_points[0]
        x_len = np.linalg.norm(x)
        x_hat = x / x_len
        p1 = np.round(calculated_points[0] + x_hat * 1/3 * x_len).astype(int)
        p2 = np.round(calculated_points[0] + x_hat * 2/3 * x_len).astype(int)
        calculated_points.append(p1)
        calculated_points.append(p2)

        x = calculated_points[2] - calculated_points[1]
        x_len = np.linalg.norm(x)
        x_hat = x / x_len
        p1 = np.round(calculated_points[1] + x_hat * 1/3 * x_len).astype(int)
        p2 = np.round(calculated_points[1] + x_hat * 2/3 * x_len).astype(int)
        calculated_points.append(p1)
        calculated_points.append(p2)

        locations_wong_order = np.array([
                selected_points[0], calculated_points[0], calculated_points[1], selected_points[1],
                calculated_points[4], calculated_points[8], calculated_points[10], calculated_points[6],
                calculated_points[5], calculated_points[9], calculated_points[11], calculated_points[7],
                selected_points[3], calculated_points[3], calculated_points[2], selected_points[2]])

        # calculate rest of points
        #print("SELECTED",selected_points)

        for p in calculated_points:
            cv2.circle(show, p, 3, (0,0,255), -1)


        cv2.imshow("frame", show)
        key = cv2.waitKey(0)
        while(key != ord(' ')):
            key = cv2.waitKey(0)

    # NOW FOR REAL CHECK 1s and 0s
    w = 5
    assert(len(locations_wrong_order) == NUM_LEDS)

    locations = locations_wong_order
    capture = cv2.VideoCapture(filename)
    show = None
    frame_number = -1
    frame_next = True
    frame_delta = 0

    center_prev = None
    recurring_center = False
    shifts = []
    autoreload = None
    prescaler = None
    mhz = None

    frame_number_100 = None
    frame_number_lsb = None
    frame_number_mhz = None
    frame_number_psc = None
    frame_number_arr = None
    frame_number_run = None

    cv2.namedWindow('frame')

    values_off = None
    values_100 = None
    values_lsb = None

    while 1:
        frame_next = False
        _, frame = capture.read()
        show = frame
        if frame is None: break
        frame_number += 1
        print(f"\rframe {frame_number}   ", end='')
        #print(frame_number, end='\r')
        values = retrieve_values(frame, locations, w)
        #print(values)
        #print(sum(values))
        # if values increase, set values 100
        if values_off is None:
            values_off = values
            #print("OFF")
        elif values_lsb is None and (values_100 is None or values_100 is not None and sum(values) > sum(values_100)):
            values_100 = values
            frame_number_100 = frame_number
            #print("100")
        elif values_lsb is None and values_100 is not None and sum(values) / sum(values_100) < 0.9: #values_100 is not None:
        #elif values_100 is not None and sum(values) < sum(values_100):
            #print("LSB")
            values_lsb = values
            frame_number_lsb = frame_number
            #print("OFF:",values_off)
            #print("100:",values_100)
            #print("LSB:",values_lsb)
            #print("DELTA",frame_number_lsb - frame_number_100)
            delta = frame_number_lsb - frame_number_100
            frame_number_mhz = frame_number_lsb + delta * 1 + delta//2
            frame_number_psc = frame_number_lsb + delta * 2 + delta//2
            frame_number_arr = frame_number_lsb + delta * 3 + delta//2
            frame_number_run = frame_number_lsb + delta * 4 + delta//2

            diff = get_diff(values_off, values_100, values)
            threshold = 10
            binary = np.where(diff > threshold, 1, 0)
            binary = binary.reshape(4, -1)

            #print("LSB VALUES:\n",binary)
            where = np.asarray(np.where(binary == 1)).T
            assert(len(where) == 2)
            corner = where[0]
            neighbor = where[1]
            if 0 < where[0][1] < binary.shape[0] and where[0][0] < binary.shape[1] - 1:
                corner = where[1]
                neighbor = where[0]

            #print("corner   =",corner)
            #print("neighbor =",neighbor)
            reference = locations.reshape(4, -1, 2)
            reordered = np.zeros((4, 4), dtype=int)
            delta_fast = neighbor - corner
            delta_fast_acc = np.zeros(2, dtype=int)
            # maybe assert delta... so one is always 0
            point = corner
            for i in range(NUM_LEDS):
                #print(point,"=",16-1-i)
                reordered[point[0]][point[1]] = NUM_LEDS-1-i
                #print("REORDERED\n",reordered)
                point += delta_fast
                delta_fast_acc += delta_fast
                # check for vertical bounds
                if not (0 <= point[0] < binary.shape[0]):
                    if delta_fast[1] == 0:
                        point -= delta_fast_acc - np.array((0, 1), dtype=int)
                    else:
                        point -= delta_fast_acc + np.array((0, 1), dtype=int)
                    delta_fast_acc = np.zeros(2, dtype=int)
                # check for horizontal bounds
                if not (0 <= point[1] < binary.shape[1]):
                    if delta_fast[0] == 0:
                        point -= delta_fast_acc - np.array((1, 0), dtype=int)
                    else:
                        point -= delta_fast_acc + np.array((1, 0), dtype=int)
                    delta_fast_acc = np.zeros(2, dtype=int)

            #print("REORDERED\n",reordered)
            binary = binary.flatten()
            reordered = reordered.flatten()
            locations = locations[reordered]
            values_off = values_off[reordered]
            values_100 = values_100[reordered]
            values_lsb = values_lsb[reordered]
            #print("FLAT\n",reordered)
            #print("BINARY FLAT",binary[reordered])
            # left-right
            #print("CORNER left:",corner_left,"/ top:",corner_top)
            #print("NEIGHBOR left:",neighbor_left,"/ top:",neighbor_top)




        binval = 0
        diff = None
        if frame_number_mhz is not None and frame_number >= frame_number_mhz:
            # convert to binary and then to number
            diff = get_diff(values_off, values_100, values)
            threshold = 10
            binary = np.where(diff > threshold, 1, 0)
            #print(binary)
            packed = np.packbits(binary)
            binval = int.from_bytes(packed.tobytes(), byteorder='big')
        if frame_number_run is not None and frame_number > frame_number_run:
            assert(diff is not None)
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
        elif frame_number_arr is not None and frame_number == frame_number_arr:
            autoreload = binval
            print(f"ARR = {autoreload}")
        elif frame_number_psc is not None and frame_number == frame_number_psc:
            prescaler = binval
            print(f"PSC = {prescaler}")
        elif frame_number_mhz is not None and frame_number == frame_number_mhz:
            mhz = binval
            print(f"MHZ = {mhz}")

        #if(values_100 is not None): print(sum(values),sum(values_100))

        cv2.imshow("frame", show)
        key = cv2.waitKey(1)
        if(key == ord('q')):
            exit(1)
        #print(V)
    print('-=-=-=-=-=-=-=-')

    # plot
    N = None
    arr = shifts
    print(f"Average shift: {np.average(arr):.4f} °")
    print(f"Total shift: {np.sum(arr):.4f} °")

    if N is not None:
        arr = scipy.signal.medfilt(arr, 2*N+1)

    assert(autoreload is not None)
    assert(prescaler is not None)
    assert(mhz is not None)

    unit = '°'
    arr_plot = arr
    if 1:
        #filtered = np.array(arr)
        filtered = -np.deg2rad(np.array(arr))
        fps_target = (1e6 * mhz / (prescaler + 1) / autoreload / 2 / NUM_LEDS)
        print("FPS_TARGET",fps_target)
        filtered = filtered / fps_target / (2*np.pi)
        fps_measured = (frame_number+1)/((frame_number+1)*(1/fps_target)+np.sum(filtered))
        print("FPS_MEASURED",fps_measured)

        # spannendere ANGABE IN:
        # fehler in ms
        # fehler in frames / fps
        # unambiguity

        #err_ideal = 1 / fps_target # measured oder target?
        omega_led_mean = (-np.array(arr) + 360 / NUM_LEDS) / (1 / (NUM_LEDS * fps_target))
        omega_led = fps_target * 360;

        # final result
        drift_ratio = (1 - omega_led_mean / omega_led)
        drift_s = drift_ratio / fps_target
        drift_us = 1000000 * drift_s
        #drift_per_1k = np.divide(1000, drift_s, out=np.zeros_like(drift_s), where=drift_s!=0)

        n_record = frame_number - frame_number_run
        t_record = n_record / fps_measured
        drift_us_per_s = 1000000 * np.sum(drift_s) / t_record;

        print(f"Calibration segment length: {t_record:.1f} s")
        print(f"Average drift from frame to frame: {np.average(drift_ratio*100):.4f} % ({np.mean(drift_s*1000):.4f} ms)")
        print(f"Total drift from frame to frame: {np.sum(drift_ratio*100):.4f} % ({np.sum(drift_s*1000):.4f} ms)")
        print(f"Drift over time : {drift_us_per_s:.4f} µs/s");
        arr_plot = drift_ratio * 100
        unit = '%'

        # PLAUSIBILITÄTSTESTS
        # oszi etc
        # abgleich mit timestamps


    arr_cumshifts = np.cumsum(arr_plot)
    #plt.plot(arr, '.')
    plt.plot(arr)
    plt.plot(arr_cumshifts)
    plt.xlabel('frame')
    plt.ylabel(f'shift [{unit}]')
    plt.grid()
    plt.show()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    #estimate("./data/72mhz-9psc-3749arr.1.mp4")
    #estimate("./data/72mhz-9psc-3749arr.2.mp4")
    estimate("./data/72mhz-9psc-3750arr.mp4")
    #estimate("./data/72mhz-9psc-3755arr.mp4")
    print("Done")

