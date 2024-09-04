import numpy as np
import cv2
import copy
from config import Config
import traceback

DEBUG_DETECT_BIT_FIELD_DISTANCES            = True
DEBUG_DETECT_BIT_FIELD_ANGLES               = True
DEBUG_DETECT_BIT_FIELD_ANGLES_COMPARE_OK    = True
DEBUG_DETECT_BIT_FIELD_ANGLES_MOST          = True
DEBUG_DETECT_BIT_FIELD_FINAL                = True
DEBUG_DETECT_BIT_FIELD_VALID_CONNECTIONS    = True

def imshow(title, img):
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.imshow(title, img)


# https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

class ImageSequence:

    def __init__(self):
        self.__frames = {}
        self.__binary_values = {}
        self.__normalized_values = {}

    # set image at certain index
    def frame_add(self, frame, real_index):
        index = int(round(real_index))
        if frame is None:
            return

        # don't add frame if already present
        if index in self.__frames:
            return

        # add frame
        self.__frames[index] = frame.copy()

    # get image at certain index
    def retrieve_at(self, real_index):
        real_index = int(round(real_index))

        #print("GETTING IMAGE @", real_index)
        if real_index in self.__frames:
            frame = self.__frames[real_index]
            return frame
        return None

    # clears the value cache (images still remain)
    def clear_cache(self):
        self.__binary_values = {}
        self.__normalized_values = {}

    # clear image sequence
    def clear(self, keep_frames=2):
        indices_keep = sorted(list(self.__frames.keys()))[-keep_frames:]
        frames_keep = [self.__frames[i] for i in indices_keep]

        self.__frames = {}
        self.__binary_values = {}
        self.__normalized_values = {}

        for i in range(len(indices_keep)):
            self.frame_add(frames_keep[i], indices_keep[i])

    # TODO: add functions to set ratios, bitfield, angles ... and cache gets cleared when setting one of them!
    # get analogue value from bitfield at certain index
    def values_from_bitfield(self, index, bitfield, ratios, angles, plus_minus=None, show=None):
        index = int(round(index))

        if plus_minus is not None:
            result = []
            for i in range(int(round(index - plus_minus)), int(round(index + plus_minus))):
                vals = self.values_from_bitfield(index, bitfield, ratios, angles)
                result.append(vals)

            medians = np.median(np.array(result), axis=0)
            return medians

        if index in self.__normalized_values:
            #print(f'INDEX {index} ALREADY EXISTS [values_from_bitfield]')
            return copy.deepcopy(self.__normalized_values[index])

        dark = self.__calibrated_dark.astype(np.float64) / 255.0
        bright = self.__calibrated_bright.astype(np.float64) / 255.0
        current = self.retrieve_at(index).copy().astype(np.float64) / 255.0

        base = np.abs(cv2.subtract(bright, dark).astype(np.float32))
        offs = np.abs(cv2.subtract(current, dark).astype(np.float32))

        # https://stackoverflow.com/questions/26248654/how-to-return-0-with-divide-by-zero
        img = np.divide(offs, base, out=np.zeros_like(offs), where=base!=0)

        size = int(round(min(ratios) * Config.EVAL_MULT))

        points_image = None
        return_image = None
        if show is not None:
            points_image = np.round(img*255).astype(np.uint8)
            mask = np.zeros(base.shape, dtype=np.uint8)
            for i, line in enumerate(bitfield):
                for j, p in enumerate(line):
                    cv2.circle(mask, p.round().astype(int), size, (255), -1)
            points_image = cv2.bitwise_and(points_image, mask)
            mask_inv = cv2.bitwise_not(mask)
            rem = cv2.bitwise_and(show, mask_inv)
            return_image = cv2.bitwise_or(points_image, rem)

            
        result = []
        for i, line in enumerate(bitfield):
            for j, p in enumerate(line):
                mask = np.zeros(base.shape, dtype=np.uint8)
                cv2.circle(mask, p.round().astype(int), size, (255), -1)
                average_color = cv2.mean(img, mask)[0]
                result.append(average_color)

        result = np.array(result)

        self.__normalized_values[index] = result
        if show is not None:
            return copy.deepcopy(result), return_image
        else:
            return copy.deepcopy(result)

    # get binary value from bitfield at certain index
    def binary_from_bitfield(self, index, bitfield, ratios, angles, plus_minus=None):
        index = int(round(index))
        if plus_minus is None:
            if index in self.__binary_values:
                return self.__binary_values[index]
            value = self.values_from_bitfield(int(round(index)), bitfield, ratios, angles)
            value = copy.deepcopy(np.where(value.reshape((Config.NUM_LEDS_EDGE, Config.NUM_LEDS_EDGE)) > 0.5, True, False))
            binval = int.from_bytes(np.packbits(value).tobytes(), byteorder='big')
            self.__binary_values[index] = binval
            return int(binval)
        else:
            values = []
            for i in range(int(round(index - plus_minus)), int(round(index + plus_minus))):
                binval = self.binary_from_bitfield(i, bitfield, ratios, angles)
                values.append(binval)
            return int(np.median(values))

    # calibrate class -> dark image (all LEDs off) at certain index
    def calibrate_dark(self, index, confidence):
        calib = []
        height, width = self.retrieve_at(index).shape
        for i in range(int(round(index - confidence)), int(round(index + confidence))):
            img = self.retrieve_at(i)
            if img is not None:
                calib.append(img)

        if len(calib) == 0:
            print('Could not find any images to calibrate!')
            raise Exception('Could not find any images to calibrate!')

        # extract and store pixel values from each image
        pixel_values = []
        norm = [img / 255.0 for img in calib]
        for img in norm:

            # convert the image to a flat array of pixel values
            pixels = np.array(img.reshape(-1))
            pixel_values.append(pixels)

        mean_pixels = np.mean(pixel_values, axis=0)
        dark = (255*mean_pixels.reshape((height, width))).round().astype(np.uint8)
        self.__calibrated_dark = dark
        self.clear_cache()

    # calibrate class -> bright image (all LEDs on) at certain index
    def calibrate_bright(self, index, confidence):
        calib = []
        height, width = self.retrieve_at(index).shape
        for i in range(int(round(index - confidence)), int(round(index + confidence))):
            img = self.retrieve_at(i)
            if img is not None:
                calib.append(img)

        if len(calib) == 0:
            print('Could not find any images to calibrate!')
            raise Exception('Could not find any images to calibrate!')

        # extract and store pixel values from each image
        pixel_values = []
        norm = [img / 255.0 for img in calib]
        for img in norm:

            # convert the image to a flat array of pixel values
            pixels = img.reshape(-1)
            pixel_values.append(pixels)

        mean_pixels = np.mean(np.array(pixel_values), axis=0)
        bright = (255*mean_pixels.reshape((height, width))).round().astype(np.uint8)
        self.__calibrated_bright = bright
        self.clear_cache()

    # detect the bit field at certain indices
    def detect_bit_field(self, index_dark, index_bright, confidence):

        #print("DETECTING...")
        # calibrate dark+bright
        self.calibrate_dark(index_dark, confidence)
        self.calibrate_bright(index_bright, confidence)

        # get difference from bright-dark
        dark = self.__calibrated_dark
        bright = self.__calibrated_bright
        diff = cv2.subtract(bright, dark)

        # apply gaussian blur
        # https://pyimagesearch.com/2021/04/28/opencv-thresholding-cv2-threshold/
        kernel = (7, 7)
        blurred = cv2.GaussianBlur(diff, kernel, 0)
        (T, thresh) = cv2.threshold(blurred, 0, 255, cv2.THRESH_OTSU)

        # TODO: make this an argument? or scale with image?
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.erode(thresh, kernel)

        # get contours
        # https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_centers = []
        min_x, max_x, min_y, max_y = (float('inf'), float('-inf'), float('inf'), float('-inf'))
        height, width = diff.shape
        for i, contour in enumerate(contours):
            x, y = contour.mean(axis=0)[0]
            contour_centers.append(np.array((x, y)))
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)

        # calculate distances
        distances = []
        distances_pairs = []
        for i, ca in enumerate(contour_centers[:-1]):
            for cb in contour_centers[i+1:]:
                distance = np.linalg.norm(ca - cb)
                distances.append(distance)
                distances_pairs.append(np.array((ca, cb)))

        if DEBUG_DETECT_BIT_FIELD_DISTANCES:
            print(f'Number of distances caluclated: {len(distances)}')
            show = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            for pair in distances_pairs:
                p0 = (int(round(pair[0][0])), int(round(pair[0][1])))
                p1 = (int(round(pair[1][0])), int(round(pair[1][1])))
                cv2.line(show, p0, p1, (0, 0, 255), 3)
                #print(a, b)
            imshow("Calculated Distances", show)

        # calculate angles between the pairs and horizontal line
        angles = []
        for i, p0 in enumerate(distances_pairs):
            l0 = p0[0] - p0[1]
            a = np.arctan2(l0[1], l0[0])
            if DEBUG_DETECT_BIT_FIELD_ANGLES:
                print(f"Angle : {l0} = {np.rad2deg(a):.2f}")

            if DEBUG_DETECT_BIT_FIELD_ANGLES:
                show = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                q0 = (int(round(p0[0][0])), int(round(p0[0][1])))
                q1 = (int(round(p0[1][0])), int(round(p0[1][1])))
                cv2.line(show, q0, q1, (0, 100, 0), 3)
                imshow("angles", show)
                cv2.waitKey(1)

            angles.append(a)

        if DEBUG_DETECT_BIT_FIELD_ANGLES:
            angless = np.sort(angles)
            for i, a in enumerate(angless):
                print(f'[{i}] = {np.rad2deg(a)}')

        # figure out angle_similar
        delta_x = np.abs(max_x - min_x)
        delta_y = np.abs(max_y - min_y)
        ratio_x = (delta_x / width)
        ratio_y = (delta_y / height)
        ratio_x_or_y = None
        angle_similar_max = Config.DETECT_ALL_ON_SIMILAR_ANGLE_MAX
        angle_similar_min = Config.DETECT_ALL_ON_SIMILAR_ANGLE_MIN
        angle_similar = None
        if ratio_x > ratio_y:
            angle_similar = (angle_similar_max - angle_similar_min) * ratio_x + angle_similar_min
            ratio_x_or_y = ratio_x
        else:
            angle_similar = (angle_similar_max - angle_similar_min) * ratio_y + angle_similar_min
            ratio_x_or_y = ratio_y
        delta_xy = delta_x / delta_y
        if DEBUG_DETECT_BIT_FIELD_ANGLES_COMPARE_OK:
            print(f'Working with angle: {np.rad2deg(angle_similar):.2f}')

        # group similar angles
        angles_similar = [[ii] for ii in range(len(angles))]
        for i, a0 in enumerate(angles):

            if DEBUG_DETECT_BIT_FIELD_ANGLES_COMPARE_OK:
                show = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

                pair = (distances_pairs[i][0], distances_pairs[i][1])
                p0 = (int(round(pair[0][0])), int(round(pair[0][1])))
                p1 = (int(round(pair[1][0])), int(round(pair[1][1])))
                cv2.line(show, p0, p1, (0, 0, 255), 3)

            for j, a1 in enumerate(angles):
                if j == i:
                    continue
                diff = np.abs(a0 - a1)
                if diff > np.pi / 2:
                    diff = np.abs(diff - np.pi)

                #show = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                if DEBUG_DETECT_BIT_FIELD_ANGLES_COMPARE_OK:
                    pair = (distances_pairs[j][0], distances_pairs[j][1])
                    p0 = (int(round(pair[0][0])), int(round(pair[0][1])))
                    p1 = (int(round(pair[1][0])), int(round(pair[1][1])))
                    cv2.line(show, p0, p1, (0, 100, 0), 1)

                if diff > angle_similar:
                    continue
                if j not in angles_similar[i]:
                    angles_similar[i].append(j)
                if DEBUG_DETECT_BIT_FIELD_ANGLES_COMPARE_OK:
                    print(f'Compare [{i} / {j}] {np.rad2deg(a0):.2f} <=> {np.rad2deg(a1):.2f} = {np.rad2deg(diff):.2f} <= {np.rad2deg(angle_similar):.2f}')
                    pair = (distances_pairs[j][0], distances_pairs[j][1])
                    p0 = (int(round(pair[0][0])), int(round(pair[0][1])))
                    p1 = (int(round(pair[1][0])), int(round(pair[1][1])))
                    cv2.line(show, p0, p1, (0, 255, 0), 3)

            if DEBUG_DETECT_BIT_FIELD_ANGLES_COMPARE_OK:
                imshow("show", show)
                cv2.waitKey(0)

        angles_desc = sorted(angles_similar, key=len, reverse=True)
        angles_desc_val = []
        for desc in angles_desc:
            angle = np.median([angles[a] for a in desc])
            angles_desc_val.append(angle)

        if DEBUG_DETECT_BIT_FIELD_ANGLES_COMPARE_OK:
            for i, angle in enumerate(angles_desc_val):
                print(f'[{angles_desc[i][0]:3}] {np.rad2deg(angle):7.2f}° (x{len(angles_desc[i])})')

        # gets the 1st most common orientation
        def get_1st(iteration, len_min):
            angle_1st = iteration
            angle_1st_val = angles_desc_val[angle_1st]
            if len(angles_desc[angle_1st]) < len_min:
                return None
            return angle_1st, angle_1st_val

        # gets the 2nd most common orientation
        def get_2nd(iteration, len_min, i1):
            angle_1st, angle_1st_val = i1
            angle_diff_min = np.deg2rad(20)
            angle_2nd_val = None
            angle_2nd = None
            iteration_current = 0

            for i, angle in enumerate(angles_desc_val):
                if np.abs(angle_1st_val - angle) >= angle_diff_min:
                    angle_2nd = i
                    angle_2nd_val = angle

                    if iteration_current >= iteration:
                        if len(angles_desc[angle_2nd]) < len_min:
                            return None
                        return angle_2nd, angle_2nd_val

                    iteration_current += 1

            return None

        # get the bitfield
        def get_bitfield(i1, i2):
            if i1 is None or i2 is None:
                return
            angle_1st, angle_1st_val = i1
            angle_2nd, angle_2nd_val = i2

            angle_diff_min = np.deg2rad(20)
            angle_2nd_val = None
            angle_2nd = None

            for i, angle in enumerate(angles_desc_val):
                if np.abs(angle_1st_val - angle) >= angle_diff_min:
                    angle_2nd = i #angles_desc[i][0]
                    angle_2nd_val = angle
                    break

            # fix angles
            if angle_1st_val < 0:
                angle_1st_val += np.pi
            if angle_2nd_val < 0:
                angle_2nd_val += np.pi
            assert(angle_1st_val >= 0 and "Should be > 0")
            assert(angle_2nd_val >= 0 and "Should be > 0")

            # determine the steeper one
            vert, horz = angle_1st, angle_2nd
            angle_1st_90 = np.abs(angle_1st_val - np.pi / 2)
            angle_2nd_90 = np.abs(angle_2nd_val - np.pi / 2)
            if angle_2nd_90 < angle_1st_90:
                vert, horz = angle_2nd, angle_1st
            vert_val = angles_desc_val[vert]
            horz_val = angles_desc_val[horz]

            #print(f'1st_90 {np.rad2deg(angle_1st_90):.2f} / 2nd_90 {np.rad2deg(angle_2nd_val):.2f}')
            #print(f'vert: {np.rad2deg(angles_desc_val[vert]):.2f}, horz: {np.rad2deg(angles_desc_val[horz]):.2f}')

            if DEBUG_DETECT_BIT_FIELD_ANGLES_MOST:
                print(f'More vertically present angle: (red) {np.rad2deg(vert_val):.2f}° / [{vert}] {np.rad2deg(angles[angles_desc[vert][0]]):.2f}° (x{len(angles_similar[angles_desc[vert][0]])})')
                print(f'More horizontally present angle: (green) {np.rad2deg(horz_val):.2f}° / [{horz}] {np.rad2deg(angles[angles_desc[horz][0]]):.2f}° (x{len(angles_similar[angles_desc[horz][0]])})')

                show = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

                for i in angles_similar[angles_desc[vert][0]]:
                    pair = distances_pairs[i]
                    p0 = (int(round(pair[0][0])), int(round(pair[0][1])))
                    p1 = (int(round(pair[1][0])), int(round(pair[1][1])))
                    cv2.line(show, p0, p1, (0, 0, 255), 3)

                for i in angles_similar[angles_desc[horz][0]]:
                    pair = distances_pairs[i]
                    p0 = (int(round(pair[0][0])), int(round(pair[0][1])))
                    p1 = (int(round(pair[1][0])), int(round(pair[1][1])))
                    cv2.line(show, p0, p1, (0, 255, 0), 3)

                imshow("most present lines", show)

            bitfield = set()
            for i in angles_similar[angles_desc[angle_1st][0]]:
                for p in (distances_pairs[i][0], distances_pairs[i][1]):
                    for j in angles_similar[angles_desc[angle_2nd][0]]:
                        for q in (distances_pairs[j][0], distances_pairs[j][1]):
                            # https://www.geeksforgeeks.org/how-to-fix-the-typeerror-unhashable-type-numpy-ndarray/
                            pt = tuple(p)
                            qt = tuple(q)
                            if pt != qt:
                                continue
                            bitfield.add(pt)
            return [p for p in bitfield]

        # prepare to get some kind of bitfield (unordered)
        len_min_1st = max(int(len(angles_desc[0]) - 1), 0)
        len_min_2nd = max(int(len(angles_desc[0]) / 2 + 0.5), 0)

        len_max_1st_index = None
        len_max_2nd_index = None
        len_max_val = None

        iteration_current_1st = 0
        iteration_current_2nd = 0

        while True:
            i1 = get_1st(iteration_current_1st, len_min_1st)
            if i1 is None:
                break

            iteration_current_2nd = 0
            iteration_current_1st += 1
            while True:
                i2 = get_2nd(iteration_current_2nd, len_min_2nd, i1)
                if i2 is None:
                    break
                iteration_current_2nd += 1
                if DEBUG_DETECT_BIT_FIELD_FINAL:
                    print(f'Iteration {iteration_current_1st} .. {iteration_current_2nd}')

                bitfield = get_bitfield(i1, i2)

                if len_max_val is None or len(bitfield) > len_max_val:
                    len_max_val = len(bitfield)
                    len_max_1st_index = i1
                    len_max_2nd_index = i2

                if len(bitfield) >= Config.NUM_LEDS:
                    break
            if len(bitfield) >= Config.NUM_LEDS:
                break

        if DEBUG_DETECT_BIT_FIELD_FINAL:
            print('Best / First good Iteration')
        bitfield = get_bitfield(len_max_1st_index, len_max_2nd_index)
        if bitfield is None:
            return None

        angle_1st, angle_1st_val = len_max_1st_index
        angle_2nd, angle_2nd_val = len_max_2nd_index
        dist_pairs_1st = [distances_pairs[p] for p in angles_desc[angle_1st]]
        dist_pairs_2nd = [distances_pairs[p] for p in angles_desc[angle_2nd]]

        if DEBUG_DETECT_BIT_FIELD_FINAL:
            show = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

            for pair in dist_pairs_1st:
                p0 = (int(round(pair[0][0])), int(round(pair[0][1])))
                p1 = (int(round(pair[1][0])), int(round(pair[1][1])))
                c1, c2, c3 = [np.random.randint(30, 256) for i in range(3)]
                cv2.line(show, p0, p1, (c3, c1, c2), 1)

            for pair in dist_pairs_2nd:
                p0 = (int(round(pair[0][0])), int(round(pair[0][1])))
                p1 = (int(round(pair[1][0])), int(round(pair[1][1])))
                c1, c2, c3 = [np.random.randint(30, 256) for i in range(3)]
                cv2.line(show, p0, p1, (c1, c2, c3), 1)

            for i, pos in enumerate(bitfield):
                print(f'[{i}] @ [{pos[0]:.2f}, {pos[1]:.2f}]')
                cv2.circle(show, (int(np.round(pos[0])), int(np.round(pos[1]))), 1, (0, 0, 255), 9)

                imshow("Possible Bitfield Positions", show)

        if len(bitfield) < Config.NUM_LEDS:
            raise Exception(f'Expected to have at least {Config.NUM_LEDS} in our bitfield; we have {len(bitfield)}')

        if DEBUG_DETECT_BIT_FIELD_FINAL:
            print(f'==> {len(bitfield)} <== Possible Bitfield Positions (may be more than {Config.NUM_LEDS}; tries to get filtered once we have the direction)')

        # get most common shortest length
        def dist_from_pair(pair):
            p0, p1 = pair
            v = p0 - p1
            return np.sqrt(v[0]**2 + v[1]**2)

        common_lengths_1st = [ dist_from_pair(ii) for ii in dist_pairs_1st ]
        common_lengths_1st = sorted(common_lengths_1st)[8:] # TODO magic number
        common_lengths_1st = [ ll for ll in common_lengths_1st if ll <= np.min(common_lengths_1st) + np.std(common_lengths_1st) / 2 ]

        common_lengths_2nd = [ dist_from_pair(ii) for ii in dist_pairs_2nd ]
        common_lengths_2nd = sorted(common_lengths_2nd)[8:] # TODO magic number
        common_lengths_2nd = [ ll for ll in common_lengths_2nd if ll <= np.min(common_lengths_2nd) + np.std(common_lengths_2nd) / 2 ]

        if DEBUG_DETECT_BIT_FIELD_VALID_CONNECTIONS:
            print("1st", common_lengths_1st)
            print("2nd", common_lengths_2nd)

        # https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
        # and IIRC the actual idea for this approach was from some other post ...
        def is_inside_rect(rect, pt):
            # corner points
            s0 = np.array(rect[0])
            s1 = np.array(rect[1])
            s2 = np.array(rect[2])
            s3 = np.array(rect[3])
            # vectors from point to corners
            v0 = s0 - pt
            v1 = s1 - pt
            v2 = s2 - pt
            v3 = s3 - pt
            # angle between each neighboring vectors
            a0 = angle_between(v0, v1)
            a1 = angle_between(v2, v1)
            a2 = angle_between(v2, v3)
            a3 = angle_between(v0, v3)
            # sum + evaluation (shouldn't compare floats for equality, hence abs and checking if below a threshold
            a = np.sum([a0,a1,a2,a3])
            d = np.abs(a - 2*np.pi)
            return d < 1e-5

        ellipse_mult = Config.ELLIPSE_MULT   # TODO: consider: image width, height
        ellipse_w = ellipse_mult * ratio_x_or_y * np.power(delta_xy, 1/5)
        ellipse_h = ellipse_mult * ratio_x_or_y / np.power(delta_xy, 1/5)

        if DEBUG_DETECT_BIT_FIELD_VALID_CONNECTIONS:
            print("delta_xy", delta_x, delta_y, delta_xy) # delta_xy larger = check in wider spots
            print("ellipse w,h", ellipse_w, ellipse_h)

        # keep common valid distances between polygons
        show = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        pairs_1st = [set() for i in range(len(bitfield))]
        pairs_2nd = [set() for i in range(len(bitfield))]
        for i, p in enumerate(bitfield):
            p0 = np.array(p)
            p1x = np.median(common_lengths_1st) * np.cos(angle_1st_val) + p0[0]
            p1y = np.median(common_lengths_1st) * np.sin(angle_1st_val) + p0[1]
            p2x = np.median(common_lengths_2nd) * np.cos(angle_2nd_val) + p0[0]
            p2y = np.median(common_lengths_2nd) * np.sin(angle_2nd_val) + p0[1]
            p1 = np.array([p1x, p1y])
            p1 = np.array([p1x, p1y])
            p2 = np.array([p2x, p2y])

            # go over 1st orientation
            for j, pn in enumerate(bitfield):
                r1_x0 = pn[0] - np.cos(angle_1st_val) * ellipse_w - np.sin(angle_1st_val) * ellipse_h
                r1_y0 = pn[1] - np.sin(angle_1st_val) * ellipse_w - np.cos(angle_1st_val) * ellipse_h
                r1_x1 = pn[0] + np.cos(angle_1st_val) * ellipse_w - np.sin(angle_1st_val) * ellipse_h
                r1_y1 = pn[1] + np.sin(angle_1st_val) * ellipse_w - np.cos(angle_1st_val) * ellipse_h
                r1_x2 = pn[0] + np.cos(angle_1st_val) * ellipse_w + np.sin(angle_1st_val) * ellipse_h
                r1_y2 = pn[1] + np.sin(angle_1st_val) * ellipse_w + np.cos(angle_1st_val) * ellipse_h
                r1_x3 = pn[0] - np.cos(angle_1st_val) * ellipse_w + np.sin(angle_1st_val) * ellipse_h
                r1_y3 = pn[1] - np.sin(angle_1st_val) * ellipse_w + np.cos(angle_1st_val) * ellipse_h
                r1 = [[r1_x0, r1_y0], [r1_x1, r1_y1], [r1_x2, r1_y2], [r1_x3, r1_y3]]
                if is_inside_rect(r1, p1):
                    if DEBUG_DETECT_BIT_FIELD_VALID_CONNECTIONS:
                        cv2.line(show, np.array([r1_x0, r1_y0]).round().astype(int), np.array([r1_x1, r1_y1]).round().astype(int), (0,0,255), 3)
                        cv2.line(show, np.array([r1_x2, r1_y2]).round().astype(int), np.array([r1_x1, r1_y1]).round().astype(int), (0,0,255), 3)
                        cv2.line(show, np.array([r1_x2, r1_y2]).round().astype(int), np.array([r1_x3, r1_y3]).round().astype(int), (0,0,255), 3)
                        cv2.line(show, np.array([r1_x0, r1_y0]).round().astype(int), np.array([r1_x3, r1_y3]).round().astype(int), (0,0,255), 3)
                        cv2.circle(show, np.array(p1).round().astype(int), 10, (127, 0, 255), 3)
                        cv2.line(show, p0.round().astype(int), p1.round().astype(int), (127, 0, 255), 3)
                        print(f"1st - Hook up {i} with {j}")
                        imshow("yay", show)
                        cv2.waitKey(0)
                    # handle all connections
                    pairs_1st[i].add(j)
                    pairs_1st[j].add(i)
                    for ii in pairs_1st[j]:
                        pairs_1st[i].add(ii)
                    for jj in pairs_1st[i]:
                        pairs_1st[j].add(jj)
                    for ii in pairs_1st[j]:
                        pairs_1st[ii].add(i)
                    for jj in pairs_1st[i]:
                        pairs_1st[jj].add(j)
                    # done; next
                    break

            # go over 2nd orientation
            for j, pn in enumerate(bitfield):
                r2_x0 = pn[0] - np.cos(angle_2nd_val) * ellipse_w - np.sin(angle_2nd_val) * ellipse_h
                r2_y0 = pn[1] - np.sin(angle_2nd_val) * ellipse_w - np.cos(angle_2nd_val) * ellipse_h
                r2_x1 = pn[0] + np.cos(angle_2nd_val) * ellipse_w - np.sin(angle_2nd_val) * ellipse_h
                r2_y1 = pn[1] + np.sin(angle_2nd_val) * ellipse_w - np.cos(angle_2nd_val) * ellipse_h
                r2_x2 = pn[0] + np.cos(angle_2nd_val) * ellipse_w + np.sin(angle_2nd_val) * ellipse_h
                r2_y2 = pn[1] + np.sin(angle_2nd_val) * ellipse_w + np.cos(angle_2nd_val) * ellipse_h
                r2_x3 = pn[0] - np.cos(angle_2nd_val) * ellipse_w + np.sin(angle_2nd_val) * ellipse_h
                r2_y3 = pn[1] - np.sin(angle_2nd_val) * ellipse_w + np.cos(angle_2nd_val) * ellipse_h
                r2 = [[r2_x0, r2_y0], [r2_x1, r2_y1], [r2_x2, r2_y2], [r2_x3, r2_y3]]
                if is_inside_rect(r2, p2):
                    if DEBUG_DETECT_BIT_FIELD_VALID_CONNECTIONS:
                        cv2.line(show, np.array([r2_x0, r2_y0]).round().astype(int), np.array([r2_x1, r2_y1]).round().astype(int), (0,255,0), 3)
                        cv2.line(show, np.array([r2_x2, r2_y2]).round().astype(int), np.array([r2_x1, r2_y1]).round().astype(int), (0,255,0), 3)
                        cv2.line(show, np.array([r2_x2, r2_y2]).round().astype(int), np.array([r2_x3, r2_y3]).round().astype(int), (0,255,0), 3)
                        cv2.line(show, np.array([r2_x0, r2_y0]).round().astype(int), np.array([r2_x3, r2_y3]).round().astype(int), (0,255,0), 3)
                        cv2.circle(show, np.array(p2).round().astype(int), 10, (0, 255, 127), 3)
                        cv2.line(show, p0.round().astype(int), p2.round().astype(int), (0, 255, 127), 3)
                        print(f"2nd - Hook up {i} with {j}")
                        imshow("yay", show)
                        cv2.waitKey(0)
                    # handle all connections
                    pairs_2nd[i].add(j)
                    pairs_2nd[j].add(i)
                    for ii in pairs_2nd[j]:
                        pairs_2nd[i].add(ii)
                    for jj in pairs_2nd[i]:
                        pairs_2nd[j].add(jj)
                    for ii in pairs_2nd[j]:
                        pairs_2nd[ii].add(i)
                    for jj in pairs_2nd[i]:
                        pairs_2nd[jj].add(j)
                    # done; next
                    break

        if DEBUG_DETECT_BIT_FIELD_VALID_CONNECTIONS:
            imshow("yay", show)

        # remove duplicate sets https://www.geeksforgeeks.org/removing-duplicates-of-a-list-of-sets-in-python/
        pairs_1st_valid = [set(s) for s in {frozenset(s) for s in pairs_1st} if len(set(s)) == Config.NUM_LEDS_EDGE]
        pairs_2nd_valid = [set(s) for s in {frozenset(s) for s in pairs_2nd} if len(set(s)) == Config.NUM_LEDS_EDGE]
        can_use_1st = (len(pairs_1st_valid) >= Config.NUM_LEDS_EDGE)
        can_use_2nd = (len(pairs_2nd_valid) >= Config.NUM_LEDS_EDGE)

        if DEBUG_DETECT_BIT_FIELD_VALID_CONNECTIONS:
            print('PAIRS_VALID 1st', pairs_1st_valid)
            print('PAIRS_VALID 2nd', pairs_2nd_valid)
            print(f'Can use --> 1st: {can_use_1st}, 2nd: {can_use_2nd}')

        if not can_use_1st and not can_use_2nd:
            input("Could not find any usable patterns (neither horizontal nor vertical lines)")
            raise Exception("Could not find any usable patterns (neither horizontal nor vertical lines)")
        if not can_use_1st or not can_use_2nd:
            # TODO add something that allows for proceeding?
            #raise Exception("Only using one orientation!")
            print("Only using one orientation!")

        def is_angle_vertical(angle):
            if 1/4*np.pi < np.abs(angle) < 3/4*np.pi:
                return True
            if 5/4*np.pi < np.abs(angle) < 7/4*np.pi:
                return True
            return False

        bitfield_lines = [bitfield]

        if can_use_1st and can_use_2nd:
            keep_1st = []
            keep_2nd = []

            # check which ones are present in first orientation
            for i, pairs in enumerate(pairs_1st_valid):
                throw_away = False
                for index in pairs:
                    exists = False
                    for pairs2 in pairs_2nd_valid:
                        if index in pairs2:
                            exists = True
                            break
                    if not exists:
                        throw_away = True
                        break
                if not throw_away:
                    keep_1st.append(i)

            # check which ones are present in second orientation
            for i, pairs in enumerate(pairs_2nd_valid):
                throw_away = False
                for index in pairs:
                    exists = False
                    for pairs2 in pairs_1st_valid:
                        if index in pairs2:
                            exists = True
                            break
                    if not exists:
                        throw_away = True
                        break
                if not throw_away:
                    keep_2nd.append(i)

            # filter
            if DEBUG_DETECT_BIT_FIELD_VALID_CONNECTIONS:
                print("KEEP 1st", keep_1st)
                print("KEEP 2nd", keep_2nd)
            pairs_1st_valid = [pairs_1st_valid[i] for i in keep_1st]
            pairs_2nd_valid = [pairs_2nd_valid[i] for i in keep_2nd]
            if len(pairs_1st_valid) != len(pairs_2nd_valid) and len(pairs_1st_valid) != Config.NUM_LEDS_EDGE:
                raise Exception(f"By now, everything we found should have given us a 'valid' pattern... lengths: [{len(pairs_1st_valid)}, {len(pairs_2nd_valid)}] are not == {Config.NUM_LEDS_EDGE}")

        if can_use_1st:
            # add the points
            bitfield_1st = []
            for i, pair in enumerate(pairs_1st_valid):
                bitfield_1st.append([])
                for j in pair:
                    bitfield_1st[i].append(bitfield[j])
                # sort via x or y axis
                if is_angle_vertical(angle_1st_val):
                    bitfield_1st[i] = np.array(sorted(bitfield_1st[i], key=lambda p: p[1]))
                else:
                    bitfield_1st[i] = np.array(sorted(bitfield_1st[i], key=lambda p: p[0]))

            if is_angle_vertical(angle_1st_val):
                # sort via x (bitfield_1st[:, 1]
                bitfield_1st = np.array(bitfield_1st)
                bitfield_1st = np.array(sorted(bitfield_1st, key=lambda p: p[0][0]))
            else:
                # sort via y (bitfield_1st[:, 1]
                bitfield_1st = np.array(bitfield_1st)
                bitfield_1st = np.array(sorted(bitfield_1st, key=lambda p: p[0][1]))

            # add to return value
            bitfield_lines.append(bitfield_1st)

        elif can_use_2nd:
            # add the points
            bitfield_2nd = []
            for i, pair in enumerate(pairs_2nd_valid):
                bitfield_2nd.append([])
                for j in pair:
                    bitfield_2nd[i].append(bitfield[j])
                # sort via x or y axis
                if is_angle_vertical(angle_2nd_val):
                    bitfield_2nd[i] = np.array(sorted(bitfield_2nd[i], key=lambda p: p[1]))
                else:
                    bitfield_2nd[i] = np.array(sorted(bitfield_2nd[i], key=lambda p: p[0]))

            if is_angle_vertical(angle_2nd_val):
                # sort via x (bitfield_2nd[:, 1]
                bitfield_2nd = np.array(bitfield_2nd)
                bitfield_2nd = np.array(sorted(bitfield_2nd, key=lambda p: p[0][0]))
            else:
                # sort via y (bitfield_2nd[:, 1]
                bitfield_2nd = np.array(bitfield_2nd)
                bitfield_2nd = np.array(sorted(bitfield_2nd, key=lambda p: p[0][1]))

            # add to return value
            bitfield_lines.append(bitfield_2nd)

        else:
            raise Exception("This should be unreachable")

        return bitfield_lines, (ratio_x, ratio_y), (angle_1st_val, angle_2nd_val)


