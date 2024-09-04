#import math
import cv2
import time
import numpy as np
import math
import copy
#import itertools
#import scipy
import matplotlib.pyplot as plt
#from cv2.typing import MatLike
from enum import Enum
from image_sequence import ImageSequence
from position_sequence import PositionSequence
from config import Config
import debug
from pathlib import Path

# https://stackoverflow.com/questions/24171064/remove-axis-label-offset-by-default
import matplotlib as mpl
mpl.rcParams['axes.formatter.useoffset'] = False

DATA_FROM = 2

SHOW_RAW = False
SHOW_COR = True

def plot(input_filename, lengths, sys_freq, arr, psc, fps_target, fps_expect, block=True):

    print('Loading files...')

    angles_cor = np.array(np.loadtxt(f'measurements/{Path(input_filename).stem}-angles_cor-{lengths}x-{arr}-{psc}-{fps_target}-{sys_freq}.txt', dtype=float))[DATA_FROM:]
    angles_raw = np.array(np.loadtxt(f'measurements/{Path(input_filename).stem}-angles_raw-{lengths}x-{arr}-{psc}-{fps_target}-{sys_freq}.txt', dtype=float))[DATA_FROM:]
    widths_cor = np.array(np.loadtxt(f'measurements/{Path(input_filename).stem}-widths_cor-{lengths}x-{arr}-{psc}-{fps_target}-{sys_freq}.txt', dtype=float))[DATA_FROM:]
    widths_raw = np.array(np.loadtxt(f'measurements/{Path(input_filename).stem}-widths_raw-{lengths}x-{arr}-{psc}-{fps_target}-{sys_freq}.txt', dtype=float))[DATA_FROM:]

    t = len(angles_cor) / fps_expect #fps_target
    t2 = len(angles_cor[1:]) / fps_expect #fps_target
    xs = np.linspace(0, t, len(angles_cor))
    xs2 = np.linspace(0, t2, len(angles_cor))

    # set up fps
    print('Evaluating mesaurements...')
    fps_target = fps_target
    fps_actual = fps_target
    if fps_target in Config.CALIBRATED_FREQUENCIES:
        fps_actual = Config.CALIBRATED_FREQUENCIES[fps_target]
        print(f'FPS calibration for {fps_target} found: {fps_actual}')
    else:
        print(f'FPS calibration not found for: {fps_target} - assuming ideal conditions')
    #fps_meas_cor = np.mean(frames_s_cor))
    #fps_meas_raw = np.mean(frames_s_raw))
    # EXPECTED SHIFT ??
    expected_shift = 2*np.pi * (1/fps_target - 1/fps_actual) * fps_target
    print(f'=> Expected shift: {np.rad2deg(expected_shift):.3f}')

    #for i in range(len(angles_cor)):
    #    shift = expected_shift * i
    #    angles_cor[i] = np.fmod(angles_cor[i] + shift, 2*np.pi)
    #    angles_raw[i] = np.fmod(angles_raw[i] + shift, 2*np.pi)
    #    val = fps_target + angles_cor[i] / (2*np.pi) / fps_target
    #    print(f'[{i}] {np.rad2deg(angles_cor[i]):.2f}° -> {np.rad2deg(np.fmod(angles_cor[i]+shift, 2*np.pi)):.2f}° (added {np.rad2deg(2*expected_shift*i):.2f}°) {val:.5f}')
    #    input('')

    angles_cor = np.fmod(angles_cor, 2*np.pi)
    angles_raw = np.fmod(angles_raw, 2*np.pi)
    #angles_cor -= angles_cor[0]
    #angles_raw -= angles_raw[0]

    #np.savetxt('angles_cor-FIXED.txt', angles_cor, fmt='%.15f')
    #np.savetxt('widths_cor-FIXED.txt', angles_raw, fmt='%.15f')

    # angles_cor
    frames_s_cor = []
    deltas_cor = [] #+ [angles_cor[i+1] - angles_cor[i] for i in range(len(angles_cor) - 1)]

    expected_shift2 = 2*np.pi * (1/fps_expect - 1/fps_target) * fps_target
    #input(f'EXPECTED SHIFT 2 {expected_shift2}')

    for i in range(len(angles_cor) - 1):
        center_prev = angles_cor[i+0]
        center = angles_cor[i+1]
        shift = None
        #print(np.round(np.rad2deg(center),2),' ', np.round(np.rad2deg(center_prev),2),end=' -> ')

        if center_prev > 0 and center > 0: # or center_prev < 0 and center < 0:
            shift = center_prev - center
            #input(f'A {np.round(np.rad2deg(shift),2)}')

        elif center_prev < 0 and center < 0:
            shift = center_prev - center
            #input(f'B {np.round(np.rad2deg(shift),2)}')

        elif center_prev > 0 and center < 0:
            #shift = center_prev - center
            l = np.sqrt(center_prev**2+center**2)
            if l >= np.pi:
                shift = - (np.pi-center_prev + np.pi+center)
            else:
                shift =  (center_prev - center)
            #input(f'E {np.round(np.rad2deg(shift),2)}')

        elif center_prev < 0 and center > 0:
            #shift = - (center_prev - center)
            l = np.sqrt(center_prev**2+center**2)
            if l >= np.pi:
                shift = (np.pi+center_prev + np.pi-center)
            else:
                shift =  (center_prev - center)
            #input(f'F {np.round(np.rad2deg(shift),2)}')


        elif center_prev < 0 and center > np.pi: # or center_prev > 0 and center < 0:
            shift = center + center_prev
            #shift = 2*np.pi - (center - center_prev)
            #shift = 2*np.pi - (center - center_prev)
            #shift = - (center + center_prev)
            #input(f'C {np.round(np.rad2deg(shift),2)}')

        else:
            shift = - (center - center_prev)
            #shift = 2*np.pi - (center - center_prev)
            #input(f'D {np.round(np.rad2deg(shift),2)}')

        #else:
        #    pass


        #input(f'Centers: {np.rad2deg(center_prev):.2f}° -> {np.rad2deg(center):.2f}° / Shift {np.rad2deg(shift):.2f}° - {np.rad2deg(expected_shift):.2f}°')
        shift += expected_shift
        shift = np.fmod(shift, 2*np.pi)
        #val = fps_target + shift / (2*np.pi) / fps_target

        shift_rotation = 2*np.pi + shift
        #delta_s = 1 / (2*np.pi / shift_rotation * fps_target)
        delta_s = 1 / (shift_rotation / (2*np.pi) * fps_target)

        # Wenn winkel grösser => bilder verloren!
        delta_s = 1/fps_expect + (1/fps_expect - delta_s)

        #print(f'SHIFT = {np.rad2deg(shift):.2f}°')
        #input(f'{shift_rotation:.2f}π / {2*np.pi:.2f}π / {fps_target:.2f} fps = {delta_s*1000:.2f} ms')

        #input(f'Shift Rotation: {np.rad2deg(shift_rotation):.2f}°  Δs {delta_s*1000:.2f} ms')

        # Wenn Winkel grösser => Bilder verloren!
        delta_s2 = 1 / fps_expect + (1/fps_expect - delta_s)
        #input(f'Shift Rotation: {np.rad2deg(shift_rotation):.2f}°  Δs {delta_s*1000:.2f}ms / {delta_s2*1000:.2f}ms  {1/delta_s:.2f}Hz / {1/delta_s2:.2f}Hz')
        #delta_s = np.mean([delta_s2,delta_s])

        frames_s_cor.append(delta_s2)
        deltas_cor.append(shift)

        #print(f'[{i}] {np.rad2deg(angles_cor[i]):.2f}° -> {np.rad2deg(np.fmod(angles_cor[i]+shift, 2*np.pi)):.2f}° (added {np.rad2deg(2*expected_shift*i):.2f}°) {val:.5f}')
        #input('')

    frames_s_raw = []
    deltas_raw = [] #+ [angles_raw[i+1] - angles_raw[i] for i in range(len(angles_raw) - 1)]

    for i in range(len(angles_raw) - 1):
        center_prev = angles_raw[i+0]
        center = angles_raw[i+1]
        shift = None
        if center_prev > 0 and center > 0 or center_prev < 0 and center < 0:
            shift = center_prev - center
        elif center_prev < 0 and center > 0 or center_prev > 0 and center < 0:
            shift = center_prev + center

        shift += expected_shift

        shift = np.fmod(shift, 2*np.pi)
        #val = fps_target + shift / (2*np.pi) / fps_target

        shift_rotation = 2*np.pi + shift
        #delta_s = 1 / (shift_rotation / (2*np.pi) * fps_target)
        delta_s = 2*np.pi / (fps_expect * (2*np.pi + shift_rotation))

        frames_s_raw.append(delta_s)
        deltas_raw.append(shift)

    deltas_raw = np.array(deltas_raw)
    deltas_cor = np.array(deltas_cor)
    frames_s_raw = np.array(frames_s_raw)
    frames_s_cor = np.array(frames_s_cor)

    widths_cor = np.array([x * frames_s_cor[i] / Config.NUM_LEDS for i, x in enumerate(widths_cor[1:])])
    widths_raw = np.array([x * frames_s_raw[i] / Config.NUM_LEDS for i, x in enumerate(widths_raw[1:])])

    #error_s_raw = 1 / fps_expect - frames_s_raw
    #error_s_cor = 1 / fps_expect - frames_s_cor

    error_s_raw = frames_s_raw - 1 / fps_expect
    error_s_cor = frames_s_cor - 1 / fps_expect

    error_cum_raw = np.cumsum(error_s_raw)
    error_cum_cor = np.cumsum(error_s_cor)

    #summm = 0
    error_delta_raw = np.array([0] + [error_cum_raw[i + 1] - error_cum_raw[i] for i in range(len(error_cum_raw) - 1)])
    error_delta_cor = np.array([0] + [error_s_cor[i + 1] - error_s_cor[i] for i in range(len(error_s_cor) - 1)])

    #for i in range(len(error_s_raw)):
    #    input(f'{np.cumsum(error_delta_raw)[i]} <==> {error_s_raw[i]}')
    #    #input(f'({xs[1:][i]} + {np.cumsum(error_s_cor)[i]}) / {xs[1:][i]} = {(xs[1:][i] + np.cumsum(error_s_cor)[i]) / xs[1:][i]}')
    #    #summm += fps_expect - 1/frames_s_raw[i]
    #    #input(f'{fps_expect} - {1/frames_s_raw[i]} => {summm}')

    ### #error_frames_raw =
    ### error_frames_cor = fps_expect - xs[1:] / ((xs[1:] + np.cumsum(error_s_cor))) * (1 / frames_s_cor)
    ### error_frames_cor2 = fps_expect - ((xs[1:] - np.cumsum(error_s_cor))) / xs[1:] * (1 / frames_s_cor)
    ### error_frames_cor3 = ((xs[1:] - np.cumsum(error_s_cor))) / xs[1:] * (1 / frames_s_cor) - fps_expect
    ### error_frames_cor4 = (xs[1:] / (xs[1:] + np.cumsum(error_s_cor))) * (1 / frames_s_cor) - fps_expect

    #error_frames_cor3 = 1/fps_expect - 1/(1/fps_expect + error_s_cor) #fps_expect - 1 / frames_s_cor

    # BELOW MIGHT BE GOOD??
    error_frames_cor3 = (1 / frames_s_cor - fps_expect) / fps_expect #/ t2 #frames_s_cor #/ fps_expect

    # calculate missed frames...
    ### #bound_upper = np.median(1/frames_s_cor) + np.std(1/frames_s_cor)
    ### fps_meas_med = np.median(1/frames_s_cor)
    ### fps_meas_mean = np.mean(1/frames_s_cor)
    ### bound_lower = 2*np.median(fps_meas_med) - 2*np.std(1/frames_s_cor) - fps_meas_mean
    ### #print("BOUNDS",bound_lower,bound_upper)
    ### missed_cor = 0
    ### missed_cor2 = 0
    ### for f in 1/frames_s_cor:
    ###     #if bound_lower < f < bound_upper:
    ###     if bound_lower < f:
    ###         continue
    ###     miss1 = 2*fps_meas_mean - fps_meas_med - f
    ###     miss2 = 2*fps_meas_med - f - fps_meas_mean
    ###     missed_cor += 1
    ###     missed_cor2 += np.ceil(np.mean([miss1,miss2]))
    ### print("MISSED",missed_cor,missed_cor2)




    #summm = 0
    #for i in range(len(error_delta_cor)):
    ##    input(f'1/({frames_s_cor[i]+error_delta_cor[i]}) = {fps_expect - 1/(frames_s_cor[i]+error_delta_cor[i])} => {summm}')
    #    summm += error_frames_cor3[i] #fps_expect - 1/(frames_s_cor[i]+error_delta_cor[i])
    #    print(f'[{i}] {np.cumsum(error_frames_cor3)[i]} .. {summm}')
    ##error_frames_cor = fps_expect - ((1/frames_s_cor -   error_s_cor))  /        * (1 / frames_s_cor)
    #print(error_frames_cor)
    #print(np.cumsum(error_frames_raw))
    #print('Lost frames:',average_frames_cor[-1])

    ####average_frames_raw = (xs[1:] - error_s_raw) / xs[1:] * np.mean(1 / frames_s_raw)
    ####average_frames_cor = (xs[1:] - error_s_cor) / xs[1:] * np.mean(1 / frames_s_raw)

    #widths_cor /= (fps_target * Config.NUM_LEDS)
    #widths_raw /= (fps_target * Config.NUM_LEDS)


    #print('')
    #if SHOW_RAW:
    #    print(f'angles_raw - min {np.rad2deg(np.min(deltas_raw)):.2f}°, max {np.rad2deg(np.max(deltas_raw)):.2f}°')
    #    print(f'angles_raw - avg {np.rad2deg(np.mean(deltas_raw)):.2f}°, std {np.rad2deg(np.std(deltas_raw)):.2f}°')
    #if SHOW_COR:
    #    print(f'angles_cor - min {np.rad2deg(np.min(deltas_cor)):.2f}°, max {np.rad2deg(np.max(deltas_cor)):.2f}°')
    #    print(f'angles_cor - avg {np.rad2deg(np.mean(deltas_cor)):.2f}°, std {np.rad2deg(np.std(deltas_cor)):.2f}°')

    print('')
    print(f'Calibration data length : {t:.2f}s')
    if SHOW_RAW:
        #print(f'angles raw - min {np.min(1/frames_s_raw):.3f}, max {np.max(1/frames_s_raw):.3f}')
        rotated = np.sum(deltas_raw)
        rotations = rotated / (2*np.pi)
        bumps = np.abs(rotations * Config.NUM_LEDS)
        bumps_freq_raw = t2 / bumps
        bumps_s_raw = 1/ bumps_freq_raw
        print(f'angles cor sum {np.rad2deg(rotated):.3f}° => approx. {rotations:.2f} rotations (may see {bumps:.1f} bumps / {bumps_s_raw:.2f} per second)')
        print(f'Expected rotation = {np.rad2deg(len(deltas_raw)*np.median(deltas_raw)):.3f}°')
    if SHOW_COR:
        #print(f'angles cor - min {np.min(1/frames_s_cor):.3f}, max {np.max(1/frames_s_cor):.3f}')
        rotated = np.sum(deltas_cor)
        rotations = rotated / (2*np.pi)
        bumps = np.abs(rotations * Config.NUM_LEDS)
        bumps_freq_cor = t2 / bumps
        bumps_s_cor = 1/ bumps_freq_cor
        print(f'angles cor sum {np.rad2deg(rotated):.3f}° => approx. {rotations:.2f} rotations (may see {bumps:.1f} bumps / {bumps_s_cor:.2f} per second)')
        #expect = len(deltas_cor)*np.median(deltas_cor)
        #print(f'Median {np.rad2deg(np.median(deltas_cor)):.3f}°, Expected rotation = {np.rad2deg(expect):.3f}°')
        #print(f'{(rotated-expect)*2*np.pi/Config.NUM_LEDS:.2f} frames delta')
    print('')

    if SHOW_RAW:
        print(f'widths raw - min {(np.min(widths_raw))*1000:.3f}ms, max {(np.max(widths_raw))*1000:.3f}ms')
        print(f'widths raw - avg {(np.mean(widths_raw))*1000:.3f}ms, std {(np.std(widths_raw))*1000:.3f}ms')
    if SHOW_COR:
        print(f'widths cor - min {(np.min(widths_cor))*1000:.3f}ms, max {(np.max(widths_cor))*1000:.3f}ms')
        print(f'widths cor - avg {(np.mean(widths_cor))*1000:.3f}ms, std {(np.std(widths_cor))*1000:.3f}ms')
    print('')

    print(f'Expecting {fps_expect} fps')
    if SHOW_RAW:
        print(f'fps raw - min {np.min(1/frames_s_raw):.3f}, max {np.max(1/frames_s_raw):.3f}')
        print(f'fps raw - avg {np.mean(1/frames_s_raw):.3f}, median {np.median(1/frames_s_raw):.3f}, std {np.std(1/frames_s_raw):.3f}')
        #print(f'About {t/(np.cumsum(error_s_raw)[-1]+t)*np.mean(1/frames_s_raw)} frames lost')
        frames_lost = t*(fps_expect-1/np.mean(frames_s_raw))
        print(f'About {np.abs(frames_lost):.1f} frames {"lost" if frames_lost > 0 else "gained"} (~= {np.abs(frames_lost)/t*60:.3f} frames/min)')
    if SHOW_COR:
        print(f'fps cor - min {np.min(1/frames_s_cor):.3f}, max {np.max(1/frames_s_cor):.3f}')
        print(f'fps cor - avg {np.mean(1/frames_s_cor):.3f}, median {np.median(1/frames_s_cor):.3f}, std {np.std(1/frames_s_cor):.3f}')
        #print(f'About {(t/(np.cumsum(error_s_cor)[-1]+t)-1)*np.mean(1/frames_s_cor)} frames lost')
        frames_lost = t*(fps_expect-1/np.mean(frames_s_cor))
        print(f'About {np.abs(frames_lost):.1f} frames {"lost" if frames_lost > 0 else "gained"} (~= {np.abs(frames_lost)/t*60:.3f} frames/min)')
    print('')


    #print(f'deltas_raw - avg {np.rad2deg(np.mean(deltas_raw)):.2f}°')
    #print(f'deltas_cor - avg {np.rad2deg(np.mean(deltas_cor)):.2f}°')
    #print(f'deltas_raw - total {np.rad2deg(np.sum(deltas_raw)):.2f}°')
    #print(f'deltas_cor - total {np.rad2deg(np.sum(deltas_cor)):.2f}°')
    #print(f'angles_raw - total {np.rad2deg(np.sum(angles_raw)):.2f}°')
    #print(f'angles_cor - total {np.rad2deg(np.sum(angles_cor)):.2f}°')

    # plot
    print('Plotting measurements...')
    ### fig, ax = plt.subplots()
    ### ax.plot(xs, np.rad2deg(deltas_cor), label='Corrected')
    ### ax.plot(xs, np.rad2deg(deltas_raw), label='Raw')
    ### #print(f'[{i}] : {values.tolist()}')
    ### #__position_sequence.set_pwm_calibration(calibrated_pwm)
    ### ax.set_xlabel('Time [s]')
    ### ax.set_ylabel('Δ Angle [°]')
    ### ax.set_title('Angle Differences')
    ### ax.grid()
    ### ax.legend()

    #frames_cum_cor = np.cumsum(frames_cum_s_cor - frames_cum_s_cor[0])
    #frames_cum_raw = np.cumsum(frames_cum_s_raw - frames_cum_s_raw[0])

    #fig, ax = plt.subplots()
    #ax.plot(xs[1:], real_fps_cor, label='Corrected')
    #ax.plot(xs[1:], real_fps_raw, label='Raw')
    #ax.set_xlabel('Time [s]')
    #ax.set_ylabel('FPS [1/s]')
    #ax.set_title('REAL Frames per Second')
    #ax.grid()
    #ax.legend()
    #plt.plot()

    #fig, ax = plt.subplots()
    #ax.plot(xs, frames_cum_s_cor, label='Corrected')
    #ax.plot(xs, frames_cum_s_raw, label='Raw')
    #ax.set_xlabel('Time [s]')
    #ax.set_ylabel('FPS [1/s]')
    #ax.set_title('Frames per Second')
    #ax.grid()
    #ax.legend()

    #fig, ax = plt.subplots()
    #ax.plot(xs, frames_cum_cor, label='Corrected')
    #ax.plot(xs, frames_cum_raw, label='Raw')
    #ax.set_xlabel('Time [s]')
    #ax.set_ylabel('FPS [1/s]')
    #ax.set_title('Frames per Second (cumulative error)')
    #ax.grid()
    #ax.legend()

    if 1:
        fig, ax = plt.subplots()
        if SHOW_COR:
            ax.plot(xs[1:], 1/frames_s_cor, label='Corrected')
        if SHOW_RAW:
            ax.plot(xs[1:], 1/frames_s_raw, label='Raw')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('FPS [Hz]')
        ax.set_title('FPS')
        ax.grid()
        ax.legend()

    if 1:
        fig, ax = plt.subplots()
        if SHOW_COR:
            ax.plot(xs[1:], frames_s_cor*1000, label='Corrected')
        if SHOW_RAW:
            ax.plot(xs[1:], frames_s_raw*1000, label='Raw')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Δ Shutter [ms]')
        ax.set_title('Δ Frame Time')
        ax.grid()
        ax.legend()

    if 1:
        fig, ax = plt.subplots()
        if SHOW_COR:
            ax.plot(xs[1:], error_s_cor*1000, label='Corrected')
        if SHOW_RAW:
            ax.plot(xs[1:], error_s_raw*1000, label='Raw')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Δ Error [ms]')
        ax.set_title('Δ Frame Time Error')
        ax.grid()
        ax.legend()

    if 1:
        fig, ax = plt.subplots()
        if SHOW_COR:
            ax.plot(xs[1:], widths_cor*1000, label='Corrected')
        if SHOW_RAW:
            ax.plot(xs[1:], widths_raw*1000, label='Raw')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Shutter [ms]')
        ax.set_title('Shutter Times')
        ax.grid()
        ax.legend()

    if 1:
        fig, ax = plt.subplots()
        if SHOW_COR:
            ax.plot(xs[1:], np.cumsum(error_s_cor), label='Corrected')
        if SHOW_RAW:
            ax.plot(xs[1:], np.cumsum(error_s_raw), label='Raw')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Error [s]')
        ax.set_title('Cumulative Frame Time Error')
        ax.grid()
        ax.legend()

    if 1:
        fig, ax = plt.subplots()
        if SHOW_COR:
            #ax.plot(xs[1:], np.cumsum(error_frames_cor), label='Corrected')
            #ax.plot(xs[1:], np.cumsum(error_frames_cor2), label='2')
            #ax.plot(xs[1:], error_frames_cor3, label='Corrected')
            ax.plot(xs[1:], np.cumsum(error_frames_cor3), label='cumsum')
            #ax.plot(xs[1:], np.cumsum(error_frames_cor4), label='4')
        if SHOW_RAW:
            ax.plot(xs[1:], np.cumsum(error_frames_raw), label='Raw')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Frames [1]')
        ax.set_title('Frames Gained/Lost over Time')
        ax.grid()
        ax.legend()

    ####fig, ax = plt.subplots()
    ####if SHOW_COR:
    ####    ax.plot(xs[1:], average_frames_cor, label='Corrected')
    ####if SHOW_RAW:
    ####    ax.plot(xs[1:], average_frames_raw, label='Raw')
    ####ax.set_xlabel('Time [s]')
    ####ax.set_ylabel('Frames [1]')
    ####ax.set_title('Averaged FPS over Time')
    #####ax.set_title('Lost Frames over Time')
    ####ax.grid()
    ####ax.legend()

    if 0:
        fig, ax = plt.subplots()
        if SHOW_COR:
            ax.scatter(np.rad2deg(angles_cor[1:]), widths_cor*1000, label='Corrected')
        if SHOW_RAW:
            ax.scatter(np.rad2deg(angles_cor[1:]), widths_raw*1000, label='Raw')
        ax.set_xlabel('Angle [°]')
        ax.set_ylabel('Width [ms]')
        ax.set_title('Angles and Widths')
        ax.grid()
        ax.legend()

    plt.show(block=block)

if __name__ == '__main__':

    #plot("752fps-600us-85k.mp4", lengths=81626, sys_freq=72, arr=747, psc=3, fps_target=752.0053475935829, fps_expect=752)
    #plot("60fps-5000us-22k.mp4", lengths=21680, sys_freq=72, arr=3177, psc=11, fps_target=58.99937067337948, fps_expect=60)
    #plot("59.9fps-4999us-at-59fps.mp4", lengths=21701, sys_freq=72, arr=3177, psc=11, fps_target=58.99937067337948, fps_expect=59.99)
    #plot('./data/60fps-nice-close-bright_1.mp4', lengths=1640, sys_freq=72, arr=3749, psc=9, fps_target=60.0, fps_expect=60)
    #plot('./data/60fps-nice-close-bright_1.mp4', lengths=1642, sys_freq=72, arr=3749, psc=9, fps_target=60.0, fps_expect=60)
    #plot("59.9fps-4999us-at-60fps.mp4", lengths=21698, sys_freq=72, arr=3749, psc=9, fps_target=60.0, fps_expect=59.99)
    #plot("59.9fps-4999us-at-61fps.mp4", lengths=21662, sys_freq=72, arr=2458, psc=14, fps_target=61.000406669377796, fps_expect=59.99)
    #plot("285fps-2055us-22k_002.mp4", lengths=21571, sys_freq=72, arr=1578, psc=4, fps_target=284.9905003166561, fps_expect=285)

    #plot('59.99fps-4999us-at-40.000000fps.avi', lengths=26661, sys_freq=72, arr=3749, psc=14, fps_target=40.0, fps_expect=59.99)
    #plot('59.99fps-4999us-at-120.000000fps.avi', lengths=26822, sys_freq=72, arr=3124, psc=5, fps_target=120.0, fps_expect=59.99)
    #plot('59.99fps-4999us-at-50.000000fps.avi', lengths=26783, sys_freq=72, arr=4499, psc=9, fps_target=50.0, fps_expect=59.99)
    #plot('59.99fps-4999us-at-75.000000fps.avi', lengths=26817, sys_freq=72, arr=3749, psc=7, fps_target=75.0, fps_expect=59.99)
    plot('59.99fps-4999us-at-56.250000fps.avi', lengths=26771, sys_freq=72, arr=3999, psc=9, fps_target=56.25, fps_expect=59.99)
    #plot('59.99fps-4999us-at-64.285713fps.avi', lengths=26921, sys_freq=72, arr=3499, psc=9, fps_target=64.28571428571429, fps_expect=59.99)

    #plot("../camera-control-pc/60fps-at-59fps.mp4", lengths=3617, sys_freq=72, arr=3177, psc=11, fps_target=58.99937067337948, fps_expect=59.99)
    #plot("60fps-wrong-arr.mp4",                     lengths=4077, sys_freq=72, arr=3755, psc=9, fps_target=59.90415335463259, fps_expect=59.99)
    #plot("./data/60fps-nice-close-bright_1.mp4",    lengths=1642, sys_freq=72, psc=9, arr=3749, fps_target=60.0, fps_expect=59.99)
    #plot("../camera-control-pc/30fps-long3.mp4",    lengths=27185,sys_freq=72, psc=11, arr=6249, fps_target=30.0, fps_expect=30)
    #plot("./data/60fps-roll-close-medium.mp4",      lengths=5559, sys_freq=72, psc=9, arr=3749, fps_target=60.0, fps_expect=59.99)

