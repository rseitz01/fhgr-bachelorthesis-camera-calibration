#include "Cam.hpp"

#include "err.h"

#include <iostream>

/*
 * is_getimageinfo
 */
//HIDS m_hCam = { 0 };

namespace Cam {

    Ueye::Ueye(void) {

    }

    Ueye::~Ueye() {
        if (m_connected) {
            this->videoStop();
            this->disconnect();
        }
    }

    int Ueye::connect(void) {
        m_error = IS_SUCCESS;
        if (!m_error && (m_error = is_InitCamera(&m_hCam, NULL))) {
            this->printError("failed opening camera");
        }
        else {
            this->printInfo("connected");
        }
        if (!m_error && (m_error = is_GetCameraInfo(m_hCam, &m_camInfo))) {
            this->printError("failed getting camera info");
        }
        if (!m_error && (m_error = is_GetSensorInfo(m_hCam, &m_sensorInfo))) {
            this->printError("failed getting sensor info");
        }

        m_roi.s32Width = m_sensorInfo.nMaxWidth;
        m_roi.s32Height = m_sensorInfo.nMaxHeight;

        if (!m_error) {
            m_connected = true;
        }
        return m_error;
    }

    int Ueye::disconnect(void) {
        if (!m_connected) return 0;
        if ((m_error = is_StopLiveVideo(m_hCam, IS_FORCE_VIDEO_STOP))) {
            this->printError("could not stop video capture");
            return -1;
        }

        if ((m_error = is_ImageQueue(m_hCam, IS_IMAGE_QUEUE_CMD_EXIT, 0, 0))) {
            this->printError("could not stop video capture");
            return -1;
        }

        if ((m_error = is_ClearSequence(m_hCam))) {
            this->printError("could not stop video capture");
            return -1;
        }

        for (int i = 0; i < static_cast<int>(m_imgMem.size()); i++) {
            char* pcMem = m_imgMem[i];
            if (m_error = is_FreeImageMem(m_hCam, pcMem, i + 1)) {
                this->printError("error while freeing image memory");
                return -1;
            }
        }

        if ((m_error = is_ExitCamera(m_hCam))) {
            this->printError("could not close camera");
            return -1;
        }
        m_hCam = 0;
        m_connected = 0;
        return 0;
    }

    int Ueye::setColorMode(int color_mode) {

        if (!m_error && (m_error = is_SetColorMode(m_hCam, color_mode))) {
            this->printError("failed opening camera");
        }
        if (!m_error && !(m_bitsPerPixel = is_SetColorMode(m_hCam, IS_GET_BITS_PER_PIXEL))) {
            this->printError("should not expect 0 bits per pixel");
        }


        return m_error;
    }

    int Ueye::setDisplayMode(int display_mode) {
        /* display_mode possible values:
        IS_SET_DM_DIB,
        IS_SET_DM_DIRECT3D
        IS_SET_DM_OPENGL
        IS_SET_DM_MONO
        IS_SET_DM_BAYER
        IS_SET_DM_YCBCR
        */

        if (!m_error && (m_error = is_SetDisplayMode(m_hCam, display_mode))) {
            this->printError("could not set image memory");
        }

        return m_error;
    }

    void Ueye::printError(std::string msg) {
        //if (m_error == IS_SUCCESS) return;
        int lastError = IS_SUCCESS;
        char* errstr = 0;
        //std::cerr << "[ERROR] " << msg << " (" << m_error << ")" << std::endl;
        is_GetError(m_hCam, &lastError, &errstr);
        //std::cerr << "        last error " << lastError << ": " << (errstr ? errstr : "(null)") << std::endl;
        THROW("%s\n        last error %u : %s", msg.c_str(), lastError, errstr);
    error: return;
    }

    void Ueye::printError(void) {
        this->printError("");
    }

    void Ueye::printWarning(std::string msg) {
        if (!m_suppress_warnings) {
            int lastError = IS_SUCCESS;
            char* errstr = 0;
            //std::cerr << "[WARNING] " << msg << " (" << m_error << ")" << std::endl;
            is_GetError(m_hCam, &lastError, &errstr);
            WARN("%s\n          last error %u : %s", msg.c_str(), lastError, errstr);
            //std::cerr << "          last error " << lastError << ": " << (errstr ? errstr : "(null)") << std::endl;
        }
    }

    void Ueye::printWarning(void) {
        this->printWarning("");
    }

    void Ueye::printInfo(std::string msg) {
        if (!m_suppress_infos) {
            //std::cerr << "[INFO] " << msg << std::endl;
            INFO("%s", msg.c_str());
        }
    }

    int Ueye::setParamPixelClock(unsigned int pixelclock_MHz, bool choose_nearest) {

        if (choose_nearest) {
            UINT nNumberOfSupportedPixelClocks = 0;
            if ((m_error = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_GET_NUMBER,
                (void*)&nNumberOfSupportedPixelClocks,
                sizeof(nNumberOfSupportedPixelClocks)))) {
                this->printWarning("could not get number of supported pixel clocks");
            } else if (nNumberOfSupportedPixelClocks > 0) {
                // No camera has more than 150 different pixel clocks.
                // Of course, the list can be allocated dynamically
                UINT nPixelClockList[150] = {0};
                long long smallest_delta = 0;
                long long smallest_i = -1;

                if ((m_error = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_GET_LIST, (void*)nPixelClockList, nNumberOfSupportedPixelClocks * sizeof(UINT)))) {
                    this->printWarning("could not get list of supported pixel clocks");
                } else {
                    for (long long i = 0; i < nNumberOfSupportedPixelClocks; i++) {
                        long long delta = pixelclock_MHz - (long long)nPixelClockList[i];
                        delta *= delta < 0 ? -1 : 1;
                        if (smallest_i == -1 || delta < smallest_delta) {
                            smallest_delta = delta;
                            smallest_i = i;
                        }
                        //printf("pixelclock : %d\n", nPixelClockList[i]);
                    }
                    if (smallest_i >= 0) {
                        pixelclock_MHz = nPixelClockList[smallest_i];
                        this->printInfo("changed pixelclock to " + std::to_string(pixelclock_MHz));
                    }
                }
            }
        }

        if ((m_error = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_SET, &pixelclock_MHz, sizeof(pixelclock_MHz)))) {
            this->printError("could not set pixel clock");
            return -1;
        }
        return 0;
    }

    int Ueye::setParamGain(unsigned int gain, bool automatic) {
        double enable = automatic;
        if ((m_error = is_SetAutoParameter(m_hCam, IS_SET_ENABLE_AUTO_GAIN, &enable, 0))) {
            this->printWarning("could not enable/disable auto gain");
        }
        if ((m_error = is_SetAutoParameter(m_hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &enable, 0))) {
            this->printWarning("could not enable/disable auto sensor gain");
        }
        if ((m_error = is_SetHardwareGain(m_hCam, IS_GET_MASTER_GAIN, gain, gain, gain))) {
            this->printError("could not set gain");
            return -1;
        }
        return 0;
    }

    int Ueye::setParamFramerate(double framerate, bool automatic) {
        double enable = automatic;
        if ((m_error = is_SetAutoParameter(m_hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &enable, 0))) {
            this->printWarning("could not enable/disable auto framerate");
        }
        if ((m_error = is_SetFrameRate(m_hCam, framerate, &framerate))) {
            this->printError("could not set framerate");
            return -1;
        } else {
            char framerate_buf[32] = { 0 };
            snprintf(framerate_buf, 32, "%.2f", framerate);
            this->printInfo("set frame rate: " + std::string(framerate_buf));
        }
        return 0;
    }

    int Ueye::setParamWhiteBalance(bool automatic) {
        return 0;
    }


    int Ueye::setParamShutterMode(int shuttermode, bool automatic) {
        double enable = automatic;
        if ((is_SetAutoParameter(m_hCam, IS_SET_ENABLE_AUTO_SHUTTER, &enable, 0))) {
            this->printWarning("could not enable/disable auto shutter");
        }
        if ((m_error = is_DeviceFeature(m_hCam, IS_DEVICE_FEATURE_CMD_SET_SHUTTER_MODE, &shuttermode, sizeof(shuttermode)))) {
            this->printError("could not set shutter mode");
            return -1;
        }
        return 0;
    }

    int Ueye::setParamRoi(int x, int y, int w, int h) {
        m_roi.s32Width = w;
        m_roi.s32Height = h;
        m_roi.s32X = x;
        m_roi.s32Y = y;
        if ((m_error = is_AOI(m_hCam, IS_AOI_IMAGE_SET_AOI, &m_roi, sizeof(m_roi)))) {
            this->printError("could not set region of interest");
            return -1;
        }
        return 0;
    }

    int Ueye::setParamExposure(double exposure_ms) {
        if ((m_error = is_Exposure(m_hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, &exposure_ms, sizeof(exposure_ms)))) {
            this->printError("could not set exposure time");
            return -1;
        }
        return 0;
    }

    int Ueye::setParamGamma(double gamma) {
        unsigned int gamma_int = (unsigned int)round(gamma) * 100;
        if ((m_error = is_Gamma(m_hCam, IS_GAMMA_CMD_SET, &gamma_int, sizeof(gamma_int)))) {
            this->printError("could not set gamma");
            return -1;
        } else {
            char gamma_buf[32] = { 0 };
            snprintf(gamma_buf, 32, "%.2f", (double)gamma_int / 100);
            this->printInfo("set gamma: " + std::string(gamma_buf));
        }
        return 0;
    }

    int Ueye::setParamOffset(int offset, bool automatic) {
        double enable = automatic;
        if ((m_error = is_SetAutoParameter(m_hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &enable, 0))) {
            this->printWarning("could not enable/disable auto whitebalance");
        }
        if ((m_error = is_SetAutoParameter(m_hCam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &enable, 0))) {
            this->printWarning("could not enable/disable auto sensor whitebalance");
        }
        if ((m_error = is_Blacklevel(m_hCam, IS_BLACKLEVEL_CMD_SET_OFFSET, &offset, sizeof(offset)))) {
            this->printError("could not set offset");
            return -1;
        }
        return 0;
    }


    int Ueye::rebuildFramebuffer(double bufferlen_s) {
        double fps = 0;
        int nBitsPerPixel = 0;
        int nColorMode = 0;
        if ((m_error = is_SetFrameRate(m_hCam, IS_GET_FRAMERATE, &fps))) {
            this->printError("could not retrieve frames per second");
            return -1;
        }
        if ((m_error = is_AOI(m_hCam, IS_AOI_IMAGE_GET_AOI, &m_roi, sizeof(m_roi)))) {
            this->printError("could not retrieve region of interest");
            return -1;
        }
        if (!(nBitsPerPixel = is_SetColorMode(m_hCam, IS_GET_BITS_PER_PIXEL))) {
            this->printError("could not retrieve bits per pixel");
            return -1;
        }
        if (!(nColorMode = is_SetColorMode(m_hCam, IS_GET_COLOR_MODE))) {
            this->printError("could not retrieve bits per pixel");
            return -1;
        }
        switch (nColorMode) {
            /*! \brief Raw sensor data, occupies 8 bits */
        case IS_CM_SENSOR_RAW8: //           11
            m_color_mode_cv = CV_8UC1;
            break;
        /*! \brief Raw sensor data, occupies 16 bits */
        case IS_CM_SENSOR_RAW10: //          33
            m_color_mode_cv = CV_16UC1;
            break;
        /*! \brief Raw sensor data, occupies 16 bits */
        case IS_CM_SENSOR_RAW12: //          27
            m_color_mode_cv = CV_16UC1;
            break;
        /*! \brief Raw sensor data, occupies 16 bits */
        case IS_CM_SENSOR_RAW16: //          29
            m_color_mode_cv = CV_16UC1;
            break;
        /*! \brief Mono, occupies 8 bits */
        case IS_CM_MONO8: //                 6
            m_color_mode_cv = CV_8UC1;
            break;
        /*! \brief Mono, occupies 16 bits */
        case IS_CM_MONO10: //                34
            m_color_mode_cv = CV_16UC1;
            break;
        /*! \brief Mono, occupies 16 bits */
        case IS_CM_MONO12: //                26
            m_color_mode_cv = CV_16UC1;
            break;
        /*! \brief Mono, occupies 16 bits */
        case IS_CM_MONO16: //                28
            m_color_mode_cv = CV_16UC1;
            break;
        /*! \brief BGR (5 5 5 1), 1 bit not used, occupies 16 bits */
        case IS_CM_BGR5_PACKED: //           (3  | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_8UC3;
            break;
        /*! \brief BGR (5 6 5), occupies 16 bits */
        case IS_CM_BGR565_PACKED: //         (2  | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_8UC3;
            break;
        /*! \brief BGR and RGB (8 8 8), occupies 24 bits */
        case IS_CM_RGB8_PACKED: //           (1  | IS_CM_ORDER_RGB)
        case IS_CM_BGR8_PACKED: //           (1  | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_8UC3;
            break;
        /*! \brief BGRA and RGBA (8 8 8 8), alpha not used, occupies 32 bits */
        case IS_CM_RGBA8_PACKED: //          (0  | IS_CM_ORDER_RGB)
        case IS_CM_BGRA8_PACKED: //          (0  | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_8UC4;
            break;
        /*! \brief BGRY and RGBY (8 8 8 8), occupies 32 bits */
        case IS_CM_RGBY8_PACKED: //          (24 | IS_CM_ORDER_RGB)
        case IS_CM_BGRY8_PACKED: //          (24 | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_8UC4;
            break;
        /*! \brief BGR and RGB (10 10 10 2), 2 bits not used, occupies 32 bits, debayering is done from 12 bit raw */
        case IS_CM_RGB10_PACKED: //          (25 | IS_CM_ORDER_RGB)
        case IS_CM_BGR10_PACKED: //          (25 | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_16UC3;
            break;
        /*! \brief BGR and RGB (10(16) 10(16) 10(16)), 6 MSB bits not used respectively, occupies 48 bits */
        case IS_CM_RGB10_UNPACKED: //        (35 | IS_CM_ORDER_RGB)
        case IS_CM_BGR10_UNPACKED: //        (35 | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_16UC3;
            break;
        /*! \brief BGR and RGB (12(16) 12(16) 12(16)), 4 MSB bits not used respectively, occupies 48 bits */
        case IS_CM_RGB12_UNPACKED: //        (30 | IS_CM_ORDER_RGB)
        case IS_CM_BGR12_UNPACKED: //        (30 | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_16UC3;
            break;
        /*! \brief BGRA and RGBA (12(16) 12(16) 12(16) 16), 4 MSB bits not used respectively, alpha not used, occupies 64 bits */
        case IS_CM_RGBA12_UNPACKED: //       (31 | IS_CM_ORDER_RGB)
        case IS_CM_BGRA12_UNPACKED: //       (31 | IS_CM_ORDER_BGR)
            m_color_mode_cv = CV_16UC3;
            break;

#if 0

        case IS_CM_JPEG: //                  32

        /*! \brief YUV422 (8 8), occupies 16 bits */
        case IS_CM_UYVY_PACKED: //           12
        case IS_CM_UYVY_MONO_PACKED: //      13
        case IS_CM_UYVY_BAYER_PACKED: //     14

        /*! \brief YCbCr422 (8 8), occupies 16 bits */
        case IS_CM_CBYCRY_PACKED: //         23

        /*! \brief RGB planar (8 8 8), occupies 24 bits */
        case IS_CM_RGB8_PLANAR: //           (1 | IS_CM_ORDER_RGB | IS_CM_FORMAT_PLANAR)

        case IS_CM_RGB12_PLANAR: //          //no compliant version
        case IS_CM_RGB16_PLANAR: //          //no compliant version
#endif
        default:
            this->printError("unknown / unsupported format");
            return -1;
        }
        int img_width = m_roi.s32Width - m_roi.s32X;
        int img_height = m_roi.s32Height - m_roi.s32Y;
        /*if ((m_error = is_ClearSequence(m_hCam))) {
            this->printError("could not clear image sequence");
            return -1;
        }*/
        m_imgMem.clear();
        for (double f = 0; f < bufferlen_s; f += 1.0 / fps) {
            size_t i = m_imgMem.size();
            //m_imgMem.resize(i + 1);
            int memId = 0;
            char* imgMem = 0; // m_imgMem.back();
            if ((m_error = is_AllocImageMem(m_hCam, img_width, img_height, nBitsPerPixel, &imgMem, &memId))) {
                this->printError("could not allocate image memory");
                return -1;
            }
            if ((m_error = is_AddToSequence(m_hCam, imgMem, memId))) {
                this->printError("could not add memory to sequence");
                return -1;
            }
            m_imgMem.push_back(imgMem);
            //printf("memids[%d] = %d\n", i, memIds[i]);
        }
        if ((m_error = is_ImageQueue(m_hCam, IS_IMAGE_QUEUE_CMD_INIT, 0, 0))) {
            this->printError("could not enable image queue");
            return -1;
        }
        return 0;
    }

    int Ueye::paramFileLoad(std::string file) {
        if ((m_error = is_ParameterSet(m_hCam, IS_PARAMETERSET_CMD_LOAD_FILE, (void *)file.c_str(), NULL))) {
            this->printError("could not load file");
            return -1;
        }
        return 0;
    }

    int Ueye::paramFileLoad(void) {
        if ((m_error = is_ParameterSet(m_hCam, IS_PARAMETERSET_CMD_LOAD_FILE, NULL, NULL))) {
            this->printError("could not load file");
            return -1;
        }
        return 0;
    }

    int Ueye::paramFileSave(std::string file) {
        if ((m_error = is_ParameterSet(m_hCam, IS_PARAMETERSET_CMD_SAVE_FILE, (void*)file.c_str(), NULL))) {
            this->printError("could not load file");
            return -1;
        }
        return 0;
    }

    int Ueye::paramFileSave(void) {
        if ((m_error = is_ParameterSet(m_hCam, IS_PARAMETERSET_CMD_SAVE_FILE, NULL, NULL))) {
            this->printError("could not load file");
            return -1;
        }
        return 0;
    }

    int Ueye::videoStart(void) {
        this->rebuildFramebuffer();
        if ((m_error = is_SetExternalTrigger(m_hCam, IS_SET_TRIGGER_OFF))) {
            this->printError("could not disable external trigger");
            return -1;
        }
        if ((m_error = is_CaptureVideo(m_hCam, IS_DONT_WAIT))) {
            this->printError("could not enable video capture");
            return -1;
        }
        if ((m_error = is_AOI(m_hCam, IS_AOI_IMAGE_GET_AOI, &m_roi, sizeof(m_roi)))) { /* query, because videoFrame requires up-to-date width/height */
            this->printError("could not retrieve region of interest");
            return -1;
        }
        memset(&m_framestats, 0, sizeof(m_framestats));
        memset(&m_framestats_prev, 0, sizeof(m_framestats_prev));
        return 0;
    }

    int Ueye::videoFrame(cv::Mat& frame, UEYE::Framestats& stats) {
        unsigned int pending = 0;
        if ((m_error = is_ImageQueue(m_hCam, IS_IMAGE_QUEUE_CMD_GET_PENDING, &pending, sizeof(pending)))) {
            this->printError("could not get pending image queue");
            return -1;
        }
        //printf("pending : %d\n", pending);

        if (pending) {
            char* pcMemPrev = 0;
            int nMemIdPrev = 0;
            int img_width = m_roi.s32Width - m_roi.s32X;
            int img_height = m_roi.s32Height - m_roi.s32Y;

            is_GetActSeqBuf(m_hCam, 0, 0, &pcMemPrev);
            is_LockSeqBuf(m_hCam, 0, pcMemPrev);
            /* read frame */
            frame = cv::Mat(img_height, img_width, m_color_mode_cv, pcMemPrev);
            /* get image info, aka. framestats */
            //memcpy(&m_framestats_prev, &m_framestats, sizeof(m_framestats));
            for (nMemIdPrev = 0; nMemIdPrev < m_imgMem.size(); nMemIdPrev++) {
                if (pcMemPrev == m_imgMem[nMemIdPrev]) {
                    nMemIdPrev++;
                    break;
                }
            }
            if (is_GetImageInfo(m_hCam, nMemIdPrev, &m_framestats.info, sizeof(m_framestats.info))) {
                this->printError("could not get image info");
                return -1;
            }
            is_UnlockSeqBuf(m_hCam, 0, pcMemPrev);

            m_framestats.frame_count++;
            m_framestats.drop_frames += (pending - 1);

            /* calculate fps */
            unsigned int second_last = m_framestats_prev.info.TimestampSystem.wSecond;
            unsigned int second_current = m_framestats.info.TimestampSystem.wSecond;
            uint64_t frame_time_last = m_framestats_prev.info.u64TimestampDevice;
            uint64_t frame_time_current = m_framestats.info.u64TimestampDevice;
            if (m_past_first_cycle && ((frame_time_current - frame_time_last) / 1e7 > 1.0)) {
                char fps_buf[32] = { 0 };
                uint64_t frames_current = m_framestats.frame_count;
                uint64_t frames_prev = m_framestats_prev.frame_count;
                double fps = 1e7 * (double)(frames_current - frames_prev) / (double)(frame_time_current - frame_time_last);
                m_framestats.fps = fps; // (m_framestats_prev.fps + 7.0 * fps) / 8.0;
                memcpy(&m_framestats_prev, &m_framestats, sizeof(m_framestats));
            } else if (!m_past_first_cycle) {
                m_past_first_cycle = true;
                memcpy(&m_framestats_prev, &m_framestats, sizeof(m_framestats));
            }
#if 0
            /* calculate approximate time of capture */
            int exposure_ms;
            if ((m_error = is_Exposure(m_hCam, IS_EXPOSURE_CMD_GET_EXPOSURE, &exposure_ms, sizeof(exposure_ms)))) {
                this->printError("could not get exposure time");
                return -1;
            }
            /* For the normal trigger mode(without overlapping trigger), the following formula can
            be used to calculate approximately the time stamp for the start of image acquisition */
            m_framestats.approx_capture_time_ms = m_framestats.info.u64TimestampDevice - static_cast<uint64_t>((frametime_inc * static_cast<double>(img_height) * 10000000.0) + (exposure_ms * 10000.0));
#endif

            /* discard pending images in queue */
            if ((m_error = is_ImageQueue(m_hCam, IS_IMAGE_QUEUE_CMD_DISCARD_N_ITEMS, &pending, sizeof(pending)))) {
                this->printError("could not discard n items in image queue");
                return -1;
            }
            memcpy(&stats, &m_framestats, sizeof(stats));

            return 0;
        }
        return -1;
    }

    int Ueye::videoStop(void) {
        if (!m_connected) return 0;
        if ((m_error = is_ImageQueue(m_hCam, IS_IMAGE_QUEUE_CMD_EXIT, 0, 0))) {
            this->printError("could not stop video capture");
            return -1;
        }

        if ((m_error = is_ClearSequence(m_hCam))) {
            this->printError("could not stop video capture");
            return -1;
        }

        if ((m_error = is_StopLiveVideo(m_hCam, IS_FORCE_VIDEO_STOP))) {
            this->printError("could not stop video capture");
            return -1;
        }
        return 0;
    }

};
