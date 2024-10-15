import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from config import Config
from scipy.fft import fft, fftfreq

import matplotlib as mpl
mpl.rcParams['axes.formatter.useoffset'] = False

#DATA_FROM = 58910 #15000 #2590 # 2
#DATA_FROM = 14200 #15000 #2590 # 2
DATA_FROM = 2

SHOW_RAW = True
SHOW_COR = True

def get_deltas(angles):
    deltas = []
    for i in range(len(angles) - 1):
        center_prev = angles[i+0]
        center = angles[i+1]
        shift = None
        #print(np.round(np.rad2deg(center),2),' ', np.round(np.rad2deg(center_prev),2),end=' -> ')

        if center_prev > 0 and center > 0: # or center_prev < 0 and center < 0:
            shift = center_prev - center

        elif center_prev < 0 and center < 0:
            shift = center_prev - center

        elif center_prev > 0 and center < 0:
            l = np.sqrt(center_prev**2+center**2)
            if l >= np.pi:
                shift = -(np.pi-center_prev + np.pi+center)
            else:
                shift = (center_prev - center)

        elif center_prev < 0 and center > 0:
            l = np.sqrt(center_prev**2+center**2)
            if l >= np.pi:
                shift = +(np.pi+center_prev + np.pi-center)
            else:
                shift = (center_prev - center)

        else:
            pass

        #input(f'[{DATA_FROM+i}] {np.rad2deg(shift):.2f}°')
        deltas.append(shift)
    return np.array(deltas)

def calculate_fps(f_K, f_G, deltas):
    delta_f = f_G - f_K
    s_K = delta_f * 2*np.pi / f_K
    s_G = delta_f * 2*np.pi / f_G
    s_V = -deltas
    delta_s = s_K - s_V
    print(f'delta_s = {np.rad2deg(np.mean(delta_s)):.2f}°')

    if delta_f != 0:
        if f_K < f_G:
            #f = (2*np.pi - delta_s) / (2*np.pi) * f_K
            f = (2*np.pi + delta_s/delta_f) / (2*np.pi) * f_K
        else:
            #f = (2*np.pi + delta_s) / (2*np.pi) * f_K
            f = (2*np.pi + delta_s/delta_f) / (2*np.pi) * f_K
    else:
        f = (2*np.pi + delta_s) / (2*np.pi) * f_K

    delta_t = 1/f - 1/f_K

    if 0:
        for i in range(len(deltas)):
            #print(f'{((np.pi*2 + delta_s[i]) / (np.pi*2) * delta_f):.2f}')
            input(f'{np.rad2deg(deltas[i]):.2f}° / {np.rad2deg(s_K):.2f}° / {np.rad2deg(s_V[i]):.2f}° / {np.rad2deg(delta_s[i]):.2f}° / {f[i]:.2f}')
            pass

    return f, delta_t

def plot(input_filename, lengths, sys_freq, arr, psc, fps_target, fps_expect, exposure_expect=None, block=True):

    # read fiels
    print('='*35)
    print(f'{Path(input_filename).stem}, {sys_freq}/{arr}/{psc}, {fps_target:.2f}Hz/{fps_expect:.2f}Hz')

    angles_cor = np.array(np.loadtxt(f'measurements/{Path(input_filename).stem}-angles_cor-{lengths}x-{arr}-{psc}-{fps_target}-{sys_freq}.txt', dtype=float))[DATA_FROM:]
    angles_raw = np.array(np.loadtxt(f'measurements/{Path(input_filename).stem}-angles_raw-{lengths}x-{arr}-{psc}-{fps_target}-{sys_freq}.txt', dtype=float))[DATA_FROM:]
    widths_cor = np.array(np.loadtxt(f'measurements/{Path(input_filename).stem}-widths_cor-{lengths}x-{arr}-{psc}-{fps_target}-{sys_freq}.txt', dtype=float))[DATA_FROM:]
    widths_raw = np.array(np.loadtxt(f'measurements/{Path(input_filename).stem}-widths_raw-{lengths}x-{arr}-{psc}-{fps_target}-{sys_freq}.txt', dtype=float))[DATA_FROM:]

    deltas_cor = get_deltas(angles_cor)
    deltas_raw = get_deltas(angles_raw)

    t = len(angles_cor) / fps_expect
    #print(f't = {t:.2f}s')
    xs = np.linspace(0, t, len(angles_cor))

    fps_actual = fps_target
    if fps_target in Config.CALIBRATED_FREQUENCIES:
        fps_actual = Config.CALIBRATED_FREQUENCIES[fps_target]
    else:
        print(f'No calibration data for "{fps_target}" found')

    # calculate actual fps etc
    f_K = fps_expect
    f_G = fps_actual

    f_cor, delta_t_cor = calculate_fps(f_K, f_G, deltas_cor)
    f_raw, delta_t_raw = calculate_fps(f_K, f_G, deltas_raw)

    widths_cor = widths_cor[1:]/np.mean(fps_actual) / Config.NUM_LEDS
    widths_raw = widths_raw[1:]/np.mean(fps_actual) / Config.NUM_LEDS
    
    # print statistics
    if SHOW_RAW:
        print(f'exposure raw - min {(np.min(widths_raw))*1000:.3f}ms, max {(np.max(widths_raw))*1000:.3f}ms')
        print(f'exposure raw - avg {(np.mean(widths_raw))*1000:.3f}ms, std {(np.std(widths_raw))*1000:.3f}ms')
    if SHOW_COR:
        print(f'exposure cor - min {(np.min(widths_cor))*1000:.3f}ms, max {(np.max(widths_cor))*1000:.3f}ms')
        print(f'exposure cor - avg {(np.mean(widths_cor))*1000:.3f}ms, std {(np.std(widths_cor))*1000:.3f}ms')

    print(f'Recording time {t/60:02.0f}:{t%60:02.0f} (mm:ss)')
    if SHOW_RAW:
        print(f'fps raw - min {np.min(f_raw):.3f}, max {np.max(f_raw):.3f}')
        print(f'fps raw - avg {np.mean(f_raw):.3f}, median {np.median(f_raw):.3f}, std {np.std(f_raw):.3f}')
        frames_lost = t*(fps_expect-np.mean(f_raw))
        print(f'About {np.abs(frames_lost):.1f} frames {"lost" if frames_lost > 0 else "gained"} (~= {np.abs(frames_lost)/t*60:.3f} frames/min)')
    if SHOW_COR:
        print(f'fps cor - min {np.min(f_cor):.3f}, max {np.max(f_cor):.3f}')
        print(f'fps cor - avg {np.mean(f_cor):.3f}, median {np.median(f_cor):.3f}, std {np.std(f_cor):.3f}')
        frames_lost = t*(fps_expect-np.mean(f_cor))
        print(f'About {np.abs(frames_lost):.1f} frames {"lost" if frames_lost > 0 else "gained"} (~= {np.abs(frames_lost)/t*60:.3f} frames/min)')
    print('')

    # plot fps over time
    if 1:
        fig, ax = plt.subplots()
        ax2 = ax.twinx()
        if SHOW_COR:
            ax.plot(xs[1:], f_cor, label='FPS')
            ax2.plot(xs[1:], np.cumsum(delta_t_cor), label='Cumulative Error', color='black')
        if SHOW_RAW:
            ax.plot(xs[1:], f_raw, label='Raw FPS', alpha=0.8)
            ax2.plot(xs[1:], np.cumsum(delta_t_raw), label='Raw Cumulative Error', color='red')
        ax.legend(loc='upper left')
        ax2.legend(loc='upper right')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('FPS [Hz]')
        ax2.set_ylabel('Error [s]')
        ax.set_title(f'FPS over Time $f$ ≈ {np.mean(f_cor):.2f} Hz ($f_K$ = {f_K:.2f} @ $f_G$ = {fps_target:.2f})')
        ax.grid()

    # plot exposure over time
    if 1:
        fig, ax = plt.subplots()
        ax2 = ax.twinx()
        if SHOW_COR:
            ax.plot(xs[1:], widths_cor*1000, label='Exposure')
            if exposure_expect is not None:
                ax2.plot(xs[1:], np.cumsum(widths_cor-exposure_expect), label='Cumulative Error', color='black')
        if SHOW_RAW:
            ax.plot(xs[1:], widths_raw*1000, label='Raw Exposure', alpha=0.8)
            if exposure_expect is not None:
                ax2.plot(xs[1:], np.cumsum(widths_raw-exposure_expect), label='Raw Cumulative Error', color='red')
        if exposure_expect is not None:
            ax2.set_ylabel('Error [s]')
            ax2.legend(loc='upper right')
        ax.legend(loc='upper left')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Exposure [ms]')
        ax.set_title(f'Exposure over Time $t$ ≈ {1000*np.mean(widths_cor):.2f}ms')
        ax.grid()

    # plot angles over exposure
    if 1:
        fig, ax = plt.subplots()
        if SHOW_COR:
            ax.scatter(np.rad2deg(angles_cor[1:]), widths_cor*1000, label='Corrected', alpha=0.2)
        if SHOW_RAW:
            ax.scatter(np.rad2deg(angles_cor[1:]), widths_raw*1000, label='Raw', alpha=0.2)
        ax.set_xlabel('Angle [°]')
        ax.set_ylabel('Exposure [ms]')
        ax.set_title('Angles vs. Exposure')
        ax.grid()
        ax.legend()

    plt.show(block=block)

if __name__ == '__main__':
    #plot(str(Path.home())+'/Videos/TEMP/20240808_160025.mp4', lengths=2591, sys_freq=72, arr=2691, psc=13, fps_target=59.70070048821906, fps_expect=60.0, exposure_expect=10000.000000000)
    plot('59.99fps-4999us-at-120.000000fps.avi', lengths=26822, sys_freq=72, arr=3124, psc=5, fps_target=120.0, fps_expect=59.99, exposure_expect=4999)
    plot('59.99fps-4999us-at-56.250000fps.avi', lengths=26771, sys_freq=72, arr=3999, psc=9, fps_target=56.25, fps_expect=59.99, exposure_expect=4999)
    plot('59.99fps-4999us-at-40.000000fps.avi', lengths=26661, sys_freq=72, arr=3749, psc=14, fps_target=40.0, fps_expect=59.99, exposure_expect=4999)
    plot('59.99fps-4999us-at-75.000000fps.avi', lengths=26817, sys_freq=72, arr=3749, psc=7, fps_target=75.0, fps_expect=59.99, exposure_expect=4999)
    plot('59.99fps-4999us-at-50.000000fps.avi', lengths=26783, sys_freq=72, arr=4499, psc=9, fps_target=50.0, fps_expect=59.99, exposure_expect=4999)
    plot('59.99fps-4999us-at-64.285713fps.avi', lengths=26921, sys_freq=72, arr=3499, psc=9, fps_target=64.28571428571429, fps_expect=59.99, exposure_expect=4999)
    plot('30fps-4999us-at-29.85fps.avi', lengths=50200, sys_freq=72, arr=5383, psc=13, fps_target=29.85035024410953, fps_expect=59.99, exposure_expect=4999)
    plot('59.99fps-4999us-at-59.700699fps.avi', lengths=108078, sys_freq=72, arr=2691, psc=13, fps_target=59.70070048821906, fps_expect=59.99, exposure_expect=4999)
    plot('59.99fps-4999us-at-60.299084fps.avi', lengths=107963, sys_freq=72, arr=4145, psc=8, fps_target=60.2990834539315, fps_expect=59.99, exposure_expect=4999)
    plot('752fps-600us-85k.avi', lengths=81626, sys_freq=72, arr=747, psc=3, fps_target=752.0053475935829, fps_expect=752.0, exposure_expect=0.000600000)
    plot('285fps-2055us-22k_002.avi', lengths=21571, sys_freq=72, arr=1578, psc=4, fps_target=284.9905003166561, fps_expect=285.0, exposure_expect=0.002055000)
    plot('60fps-5000us-22k.avi', lengths=21680, sys_freq=72, arr=3177, psc=11, fps_target=58.99937067337948, fps_expect=59.99, exposure_expect=0.005000000)
    plot('59.9fps-4999us-at-61fps.avi', lengths=21662, sys_freq=72, arr=2458, psc=14, fps_target=61.000406669377796, fps_expect=59.99, exposure_expect=0.004999000)
    plot('59.9fps-4999us-at-60fps.avi', lengths=21698, sys_freq=72, arr=3749, psc=9, fps_target=60.0, fps_expect=59.99, exposure_expect=0.004999000)
    plot('59.9fps-4999us-at-59fps.avi', lengths=21701, sys_freq=72, arr=3177, psc=11, fps_target=58.99937067337948, fps_expect=59.99, exposure_expect=0.004999000)
    plot('59.99fps-4999us-at-59.700699fps-blurry.avi', lengths=25663, sys_freq=72, arr=2691, psc=13, fps_target=59.70070048821906, fps_expect=59.99, exposure_expect=0.005000000)
    plot('20240808_160025.mp4', lengths=2591, sys_freq=72, arr=2691, psc=13, fps_target=59.70070048821906, fps_expect=60.0, exposure_expect=12500.000000000)

