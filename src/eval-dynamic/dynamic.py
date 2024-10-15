#import math
import traceback
import cv2
import time
import numpy as np
import math
import copy
import matplotlib.pyplot as plt
from enum import Enum
from image_sequence import ImageSequence
from position_sequence import PositionSequence
from config import Config
import debug
from pathlib import Path
from plotter import plot

import threading
import queue

# https://stackoverflow.com/questions/24171064/remove-axis-label-offset-by-default
import matplotlib as mpl
mpl.rcParams['axes.formatter.useoffset'] = False


selected_points = []

def imshow(title, img):
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.imshow(title, img)

def mouse_callback(event, x, y, flags, param):
    global selected_points
    if event==cv2.EVENT_LBUTTONDOWN:
        selected_points.append(np.array([x,y]))


class Analyzer:

    class Step(Enum):
        NONE = 0
        DETECT_ALL_ON_FIRST_BEGIN = 1
        DETECT_ALL_ON_FIRST_END = 2
        DETECT_BIT_FIELD = 3
        DETECT_ORDER = 4
        DETECT_ALL_OFF_PWM_BEGIN = 5
        DETECT_PWM_PHASES = 6
        DETECT_RIDER = 10
        QUIT = 9999

    def __init__(self, plot=True):
        # obscure stuff
        self.__step = Analyzer.Step.NONE
        self.__time_prev = None
        self.__time_next = None
        self.__time_start = None
        self.__capture = None
        self.__skip_capture = False
        self.__frame_number = 0
        self.__frame_show = None
        self.__quit = False
        self.__window_title = 'frame'
        self.__index_all_on_begin = None
        self.__index_all_on_end = None
        self.__index_order = None
        self.__index_sys_freq = None
        self.__index_arr = None
        self.__index_psc = None
        self.__index_pwm_count = None
        self.__index_rider = None
        self.__unordered_bitfields = None
        self.__wait_space = False
        self.__values_between_offstates = None
        self.__pwm_expecting_finish = False
        self.__image_sequence = ImageSequence()
        self.__position_sequence = PositionSequence()
        self.__pwm_indices = None
        self.__delta4 = None
        self.__frame_max = None
        self.exposure_expect = None
        self.__plot = plot
        # "normal" stuff
        self.sys_freq = None
        self.arr = None
        self.psc = None
        self.pwm_count = None
        self.fps_expect = None
        # other stuff
        if plot:
            cv2.namedWindow(self.__window_title, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(self.__window_title, mouse_callback)

    def load_file(self, filename, fps_expect, exposure_expect=None):
        self.input_filename = filename
        self.fps_expect = fps_expect
        if exposure_expect:
            self.exposure_expect = exposure_expect
        self.__capture = cv2.VideoCapture(filename)
        if not self.__capture.isOpened():
            raise Exception(f'Could not open: {filename}')
        self.__step = Analyzer.Step.DETECT_ALL_ON_FIRST_BEGIN
        self.__frame_max = int(self.__capture.get(cv2.CAP_PROP_FRAME_COUNT))

    def step_process(self):
        #print("STEP PROCESS...")
        if self.__quit:
            return None

        frame_next = self.__image_sequence.retrieve_at(self.__frame_number - 0)
        frame_prev = self.__image_sequence.retrieve_at(self.__frame_number - 1)
        #print(self.__step.name)
        if (frame_next is None or frame_prev is None) and self.__step != Analyzer.Step.QUIT:
            if self.__plot:
                print(f"NO FRAMES @ {self.__frame_number-0} or {self.__frame_number-1} ({frame_next is None}, {frame_prev is None})")
            return

        elif self.__step == Analyzer.Step.DETECT_ALL_ON_FIRST_BEGIN:
            self.__wait_space = False
            diff = cv2.subtract(frame_next, frame_prev)
            self.__frame_show = diff
            self.__frame_show = frame_next
            std = np.std(diff)
            if std >= Config.DETECT_ALL_ON_FIRST_THRESHOLD:
                self.__index_all_on_begin = self.__frame_number
                self.__step = Analyzer.Step.DETECT_ALL_ON_FIRST_END
                #self.__wait_space = True
                if self.__plot:
                    print('DETECT_ALL_ON_BEGIN    ok')

        elif self.__step == Analyzer.Step.DETECT_ALL_ON_FIRST_END:
            self.__wait_space = False
            diff = cv2.subtract(frame_prev, frame_next)
            self.__frame_show = diff
            std = np.std(diff)
            if std >= Config.DETECT_ALL_ON_FIRST_END_THRESOLD:
                self.__index_all_on_end = self.__frame_number
                #self.__wait_space = True
                if self.__plot:
                    print('DETECT_ALL_ON_END    ok')

                index_delta = self.__index_all_on_end - self.__index_all_on_begin
                index_dark = self.__index_all_on_begin - index_delta / 2
                index_bright = self.__index_all_on_begin + index_delta / 2
                self.__index_order = index_delta + index_bright
                if self.__plot:
                    print(f'    => {index_dark} index dark\n'
                          f'    => {index_bright} index bright\n'
                          f'    => {self.__index_order} index order\n'
                          )

                try:
                    self.__unordered_bitfields, self.__ratio_scale, self.__angles = self.__image_sequence.detect_bit_field(index_dark, index_bright, int(round(index_delta / 4)))

                    # next step
                    self.__step = Analyzer.Step.DETECT_ORDER
                    self.__skip_capture = True
                except Exception:
                    self.__step = Analyzer.Step.DETECT_ALL_ON_FIRST_BEGIN
                    self.__frame_number = self.__index_all_on_begin


        elif self.__step == Analyzer.Step.DETECT_ORDER:
            self.__wait_space = False
            if self.__frame_number >= self.__index_order:
                if self.__plot:
                    print('DETECT_ORDER     ok')

                state = self.__image_sequence.retrieve_at(self.__index_order)
                self.__frame_show = state.copy()

                def find_orientation_x(values_arg):
                    values = values_arg
                    index = None
                    rotate = False
                    compare = Config.LED_PATTERN_ORDER[Config.NUM_LEDS_EDGE - 1]
                    # get x orientation
                    for i, line in enumerate(values):
                        if (line == compare).all():
                            if index is None:
                                index = i
                            else:
                                raise Exception("Should not find pattern ONE twice or more!")

                    values = np.rot90(values)
                    for i, line in enumerate(values):
                        if (line == compare).all():
                            if index is None:
                                index = i
                                rotate = True
                            else:
                                raise Exception("Should not find pattern ONE twice or more!")

                    return index, rotate

                def find_orientation_y(values_arg):
                    values = values_arg
                    index = None
                    flip = False
                    compare = Config.LED_PATTERN_ORDER[0]

                    #print(values)
                    for col in range(values.shape[1]):
                        line = values[::-1, col]
                        #print("1", line)
                        if (line == compare).all():
                            if index is None:
                                index = col
                            else:
                                raise Exception("Should not find pattern TWO twice or more!")

                    values = np.flip(values, axis=0)
                    #print(values)
                    for col in range(values.shape[1]):
                        line = values[::-1, col]
                        #print("2", line)
                        if (line == compare).all():
                            if index is None:
                                index = col
                                flip = True
                            else:
                                raise Exception("Should not find pattern TWO twice or more!")

                    return index, flip

                def find_remaining(values):
                    result = []
                    for i in range(Config.LED_PATTERN_ORDER.shape[1]):
                        if i == 0 or i + 1 == Config.NUM_LEDS_EDGE:
                            continue
                        compare = Config.LED_PATTERN_ORDER[::-1, i]
                        index = None
                        for col in range(values.shape[1]):
                            line = values[::-1, col]
                            if (line == compare).all():
                                if index is None:
                                    index = col
                                else:
                                    raise Exception(f"Should not find pattern [{i}] twice or more!")
                        if index is None:
                            raise Exception(f"Did not find any pattern matching index [{i}]")
                        result.append(index)
                    return result

                def orient_bitfield(nb):
                    if self.__unordered_bitfields[nb] is None:
                        return

                    bitfield = copy.deepcopy(np.array(self.__unordered_bitfields[nb]))
                    # get values
                    values = self.__image_sequence.values_from_bitfield(self.__index_order, bitfield, self.__ratio_scale, self.__angles)
                    values = np.where(values.reshape((Config.NUM_LEDS_EDGE, Config.NUM_LEDS_EDGE)) > 0.5, True, False)

                    ###print('VALUES:\n',values.astype(int))

                    orientation_x, rotate_x = find_orientation_x(values)
                    if rotate_x:
                        values = np.rot90(values)

                    orientation_y, flip_y = find_orientation_y(values)
                    if flip_y:
                        values = np.flip(values, axis=0)

                    rest = find_remaining(values)

                    # fix bitfield
                    if rotate_x:
                        bitfield = np.rot90(bitfield)
                        ###print("Rotated...")
                    if flip_y:
                        bitfield = np.flip(bitfield, axis=0)
                        ###print("Flipped...")

                    indices = [orientation_y] + rest[::-1]
                    indices.append([x for x in list(range(Config.NUM_LEDS_EDGE)) if x not in indices][0])
                    indices = indices[::-1]

                    bitfield_indices = np.rot90(bitfield, k=3)
                    bitfield_fixed = [bitfield_indices[i] for i in indices]
                    bitfield_fixed = np.rot90(bitfield_fixed, k=1)

                    values_indices = np.rot90(values, k=3)
                    values_fixed = [values_indices[i] for i in indices]
                    values_fixed = np.rot90(values_fixed, k=1)

                    self.__unordered_bitfields[nb] = np.array(bitfield_fixed)
                    self.__image_sequence.clear_cache()
                    binval = self.__image_sequence.binary_from_bitfield(self.__index_order, bitfield_fixed, self.__ratio_scale, self.__angles)

                    if binval != Config.LED_PATTERN_NUMBER:
                        raise Exception(f"Wrong pattern number, expected {Config.LED_PATTERN_NUMBER:x}, got {binval:x}")

                    # draw the nth orientation
                    for lines in bitfield:
                        for i, p in enumerate(lines[:-1]):
                            p1 = np.array(p).round().astype(int)
                            p2 = np.array(lines[i+1]).round().astype(int)
                            cv2.line(self.__frame_show, p1, p2, (i*20+50), 3)


                for i, p in enumerate(self.__unordered_bitfields[0]):
                    x = int(round(p[0]))
                    y = int(round(p[1]))
                    val = state[y, x]
                    if debug.DETECTED_BITFIELD_POSITIONS:
                        print(f'{i} @ [{x}, {y}] = {val}')
                    cv2.circle(self.__frame_show, (x, y), 3, (127), -1)
                    #cv2.waitKey(0)

                orient_bitfield(1)

                # next step
                self.__step = Analyzer.Step.DETECT_ALL_OFF_PWM_BEGIN

        elif self.__step == Analyzer.Step.DETECT_ALL_OFF_PWM_BEGIN:
            self.__wait_space = False

            assert(self.__unordered_bitfields[1] is not None)

            # get binary value and save
            binval = self.__image_sequence.binary_from_bitfield(self.__frame_number, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles)
            if self.__values_between_offstates is None:
                self.__values_between_offstates = []
            self.__values_between_offstates.append(binval)

            if binval == 0x0:
                if self.__plot:
                    print('DETECT_ALL_OFF_PWM_BEGIN     ok')

                self.__index_pwm_all_off = self.__frame_number
                # gather the indices
                delta_indices = self.__frame_number - self.__index_all_on_end # all_on_end is synonymous with order_begin
                delta_steps = 5
                delta = delta_indices / delta_steps
                delta2 = delta/2
                self.__delta4 = delta2/2
                self.__index_sys_freq = self.__index_all_on_end + (1 * delta + delta2)
                self.__index_arr = self.__index_all_on_end + (2 * delta + delta2)
                self.__index_psc = self.__index_all_on_end + (3 * delta + delta2)
                self.__index_pwm_count = self.__index_all_on_end + (4 * delta + delta2)

                self.sys_freq = self.__image_sequence.binary_from_bitfield(self.__index_sys_freq, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles, delta2/2)
                self.arr = self.__image_sequence.binary_from_bitfield(self.__index_arr, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles, delta2/2)
                self.psc = self.__image_sequence.binary_from_bitfield(self.__index_psc, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles, delta2/2)
                self.pwm_count = self.__image_sequence.binary_from_bitfield(self.__index_pwm_count, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles, delta2/2)
                self.fps_target = (1e6 * self.sys_freq / (self.psc + 1) / (self.arr + 1) / 2 / Config.NUM_LEDS)

                if self.__plot:
                    print(f'    => {int(self.__index_order)} index order\n'
                        f'    => {int(self.__index_sys_freq)} index sys_freq [{self.sys_freq}]\n'
                        f'    => {int(self.__index_arr)} index arr [{self.arr}]\n'
                        f'    => {int(self.__index_psc)} index psc [{self.psc}]\n'
                        f'    => {int(self.__index_pwm_count)} index pwm_count [{self.pwm_count}]\n'
                        f'    => fps ~= [{self.fps_target:.3f}]\n'
                        )

                # done, next step
                self.__step = Analyzer.Step.DETECT_PWM_PHASES
                self.__wait_space = False

        elif self.__step == Analyzer.Step.DETECT_PWM_PHASES:
            self.__wait_space = False
            values = self.__image_sequence.values_from_bitfield(self.__frame_number, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles)
            value = self.__image_sequence.binary_from_bitfield(self.__frame_number, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles)

            #print(np.mean(values))
            if np.mean(values) >= 0.55:
                self.__pwm_expecting_finish = True

            if self.__pwm_expecting_finish and value == 0:
                if self.__plot:
                    print('DETECT_PWM_PHASES     ok')

                delta_indices = self.__frame_number - self.__index_pwm_all_off
                delta = delta_indices / (self.pwm_count + 1)
                delta2 = delta/2
                delta4 = delta2/2

                self.__pwm_indices = np.linspace(self.__index_pwm_all_off + delta2, self.__index_pwm_all_off - delta2 + delta * (self.pwm_count + 1), self.pwm_count + 1)
                self.__index_rider = self.__index_pwm_all_off + delta * (self.pwm_count + 2) # TODO this might not work for specific FPSs?
                if self.__plot:
                    print("     => PWM INDICES = ", self.__pwm_indices.round().astype(int))
                    print(f'    => {int(self.__index_rider)} index rider\n')

                self.__image_sequence.calibrate_dark(self.__pwm_indices[0], delta4)
                self.__image_sequence.calibrate_bright(self.__pwm_indices[-1], delta4)

                #percentages = np.linspace(0, 100, self.pwm_count + 1)
                percentages = 100*np.array([(Config.CALIBRATED_PWM[x][0]-Config.CALIBRATED_PWM[x][1]) / Config.CALIBRATED_PWM[x][0] for x in range(len(Config.CALIBRATED_PWM))])
                plotdata = []
                for i, index in enumerate(self.__pwm_indices):
                    values = self.__image_sequence.values_from_bitfield(index, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles, delta4)
                    plotdata.append(values)

                plotdata = np.array(plotdata)
                assert(plotdata.shape[1] == Config.NUM_LEDS)

                calibrated_pwm = []
                for led in range(Config.NUM_LEDS):
                    y = plotdata[:, led]
                    calibrated_pwm.append(y)

                self.__position_sequence.set_pwm_calibration(calibrated_pwm)

                if self.__plot:
                    fig, ax = plt.subplots()

                    ytest = np.linspace(0, 1, 1001)
                    calibrated_interpolated = self.__position_sequence.get_pwm_interpolation(ytest)
                    for led in range(Config.NUM_LEDS):
                        # https://stackoverflow.com/questions/28999287/generate-random-colors-rgb
                        color = Config.LED_COLORS[led] #tuple(np.random.uniform(0, 0.5, size=3))
                        # plot
                        y = plotdata[:, led]
                        ax.scatter(percentages, y, label=f'LED [{led}]', color=color)
                        ax.plot(100*calibrated_interpolated[led], ytest, label=f'INT [{led}]', color=color, linestyle='dashed')

                    ax.set_xlabel('PWM [%]')
                    ax.set_ylabel('Intensity [0..1]')
                    ax.set_title('PWM Curve')
                    ax.grid()
                    ax.legend()
                    plt.show(block=False)

                # done, next step
                self.__step = Analyzer.Step.DETECT_RIDER
                self.__image_sequence.clear()
                self.__wait_space = False
            pass

        elif self.__step == Analyzer.Step.DETECT_RIDER:
            self.__wait_space = False
            if self.__frame_number >= self.__index_rider:
                self.__wait_space = False

                values, self.__frame_show = self.__image_sequence.values_from_bitfield(self.__frame_number, self.__unordered_bitfields[1], self.__ratio_scale, self.__angles, show=self.__frame_show)
                angle = self.__position_sequence.angle_from_values(values, self.__frame_number)
                width = self.__position_sequence.width_from_values(values, self.__frame_number)
                info = f'  {np.rad2deg(angle).round(1):-5f}° {width/self.fps_target/Config.NUM_LEDS*1000:.3f}ms  '

                if self.__plot:
                    print(info, end='')

                info = f'  {np.rad2deg(angle).round(1)}deg {width/self.fps_target/Config.NUM_LEDS*1000:.3f}ms  '
                cv2.putText(self.__frame_show, info, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

                self.__image_sequence.clear()

        elif self.__step == Analyzer.Step.QUIT:
            # gather results
            print('Quitting -> check to see if we got data...')
            angles = self.__position_sequence.angles_stored()
            widths = self.__position_sequence.widths_stored()
            if angles is not None:
                print('Got data...')
                angles_cor, angles_raw = angles
                widths_cor, widths_raw = widths
                assert(len(angles_cor) == len(angles_raw))
                assert(len(widths_cor) == len(widths_raw))
                assert(len(angles_cor) == len(widths_cor))

                print('Saving to files...')
                np.savetxt(f'measurements/{Path(self.input_filename).stem}-angles_cor-{len(angles_cor)}x-{self.arr}-{self.psc}-{self.fps_target}-{self.sys_freq}.txt', angles_cor, fmt='%.15f')
                np.savetxt(f'measurements/{Path(self.input_filename).stem}-angles_raw-{len(angles_raw)}x-{self.arr}-{self.psc}-{self.fps_target}-{self.sys_freq}.txt', angles_raw, fmt='%.15f')
                np.savetxt(f'measurements/{Path(self.input_filename).stem}-widths_cor-{len(widths_cor)}x-{self.arr}-{self.psc}-{self.fps_target}-{self.sys_freq}.txt', widths_cor, fmt='%.15f')
                np.savetxt(f'measurements/{Path(self.input_filename).stem}-widths_raw-{len(widths_raw)}x-{self.arr}-{self.psc}-{self.fps_target}-{self.sys_freq}.txt', widths_raw, fmt='%.15f')

                plotcommand = f'plot(\'{self.input_filename}\', lengths={len(angles_cor)}, sys_freq={self.sys_freq}, arr={self.arr}, psc={self.psc}, fps_target={self.fps_target}, fps_expect={self.fps_expect}'
                if self.exposure_expect is not None:
                    plotcommand += f', exposure_expect={self.exposure_expect:.9f}'
                plotcommand += ')'
                print(plotcommand)
                with open('measurements/plotcommands.txt', 'a') as file:
                    file.write(f'{plotcommand}\n')

                if self.__plot:
                    if self.exposure_expect is not None:
                        plot(self.input_filename, lengths=len(angles_cor), sys_freq=self.sys_freq, arr=self.arr, psc=self.psc, fps_target=self.fps_target, fps_expect=self.fps_expect, block=False, exposure_expect=self.exposure_expect)
                    else:
                        plot(self.input_filename, lengths=len(angles_cor), sys_freq=self.sys_freq, arr=self.arr, psc=self.psc, fps_target=self.fps_target, fps_expect=self.fps_expect, block=False)

                cv2.waitKey(0)
                self.__position_sequence.clear()

                # idea: generate a PDF report!
            else:
                self.__quit = True

    def frame_next(self):
        if self.__quit or self.__step == Analyzer.Step.QUIT:
            return None
        if not self.__capture.isOpened():
            return None
        self.__frame_number += 1
        frame = self.__image_sequence.retrieve_at(self.__frame_number)
        if frame is None:
            _, frame = self.__capture.read()
            if frame is None:
                return None
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.__image_sequence.frame_add(frame, self.__frame_number)

        self.__frame_show = frame
        return self.__frame_number

    def frame_progress(self, show=True, console=True, handle_input=True, wait_space=False):
        if console:
            info = f"\rframe {self.__frame_number:6} / {self.__frame_max:6}   "
            if self.__time_start is not None and self.__time_prev is not None and self.__time_next is not None:
                steps_left = self.__frame_max - self.__frame_number
                time_left1 = (self.__time_next - self.__time_start) / self.__frame_number * steps_left
                time_left2 = (self.__time_next - self.__time_prev) * steps_left
                time_left3 = abs(time_left1 - time_left2)
                info += f' (est mm:ss {time_left1/60:02.0f}:{np.fmod(time_left1,60):02.0f}±{time_left3/60:02.0f}:{np.fmod(time_left3, 60):02.0f})    '
            if self.__plot:
                print(info, end='')
            else:
                if not ((self.__frame_number+1) % 25):
                    print(info)
        if show:
            if self.__plot:
                # handle input
                wait = 0 if wait_space or self.__wait_space else 1
                while wait or wait_space or self.__wait_space:
                    self.__key = cv2.waitKey(wait)
                    # pressing q will quit
                    if self.__key == ord('q'):
                        if self.__step == Analyzer.Step.QUIT:
                            self.__quit = True
                        self.__step = Analyzer.Step.QUIT
                        break
                    # in any case we press space, continue on... (may be waiting indefinitely)
                    if self.__key == ord(' '):
                        break
                    # if we waited a millisecond, we're done anyways
                    if wait:
                        break

    def frame_show(self):
        if self.__plot:
            imshow("frame", self.__frame_show)

    def process(self):
        self.__time_start = time.time()
        while not self.__quit:
            #print(self.__step.name)
            self.__time_prev = self.__time_next
            self.__time_next = time.time()
            #print("FRAME_NEXT")
            frame = self.frame_next()
            if frame is None:
                self.__step = Analyzer.Step.QUIT
            self.frame_progress()#console=self.__plot)
            self.step_process()
            self.frame_show()#console=self.__plot)


if __name__ == '__main__':

    # files to be analyzed
    files = [
             # path (required), expected fps (required), expected exposure (optional, recommended)
             (str(Path.home())+'/Videos/TEMP/20240808_160025.mp4', 60.0, 12500),
            ]


    # queue and thread it
    q = queue.Queue(maxsize=0)
    num_threads = 10
    if len(files) == 1:
        num_threads = 1

    # worker for if we thread it
    def worker(q):
        while True:
            file = q.get()
            print(f'Working on {file}')
            try:
                analyzer = Analyzer(plot=do_plot)
                analyzer.load_file(file[0], file[1], file[2])
                analyzer.process()
            except Exception as e:
                print(f"Failed {file}: {e}")
                print(traceback.format_exc())
            print(f'Done with {file}')
            q.task_done()

    do_plot = False
    if num_threads == 1:
        # don't thread if only one file
        do_plot = True
        file = files[0]
        analyzer = Analyzer(plot=do_plot)
        analyzer.load_file(file[0], file[1], file[2])
        analyzer.process()
    else:
        for _ in range(num_threads):
            work = threading.Thread(target=worker, args=(q,))
            work.setDaemon(True)
            work.start()

        for file in files:
            q.put(file)

        print(f'Joining..')
        q.join()

    print("Done")
