#include <cstdio>
#include <cstdlib>

#include "Cam.hpp"
#include "err.h"
#include "opencv2/videoio.hpp"

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utils/logger.hpp>


#ifndef TTY_AUTO_ENABLED
#define TTY_AUTO_ENABLED    1
#endif


#if TTY_AUTO_ENABLED

#include "tty.hpp"

#define TTY_PORTNAME        "/dev/ttyACM0"

#endif


int main(void)
{
    int err = 0;
    int fd = 0;
    bool alive = true;
    bool video = false;

    double fps = 60.0;
    double fps_device = fps + 0.3;

    const char *all_on = "mode display=1111111111111111\n";
    const char *all_off = "mode none\n";

    cv::Mat outimg;
	Cam::UEYE::Framestats stats;
    Cam::Ueye cam = {};
	cv::Mat frame = cv::Mat::zeros(10, 10, CV_8UC1);
    std::string filename;
    cv::VideoWriter writer;


    std::vector<std::string> tty_auto_commands = {
        std::string("config fps=") + std::to_string(fps_device),
        //std::string("config arr=3755"),
        std::string("mode auto"),
    };

    TRY(cam.connect(), "could not connect with camera");

    std::cout << "filename (without extension): ";
    std::getline(std::cin, filename);
    filename += ".mp4";
    //std::string filename = "video.mp4";
    writer = cv::VideoWriter(filename, cv::VideoWriter::fourcc('m','p','4','v'), fps, cv::Size(1600, 1200), false);

	cam.setColorMode(IS_CM_MONO8); // IS_CM_BGR8_PACKED);
	cam.setDisplayMode(IS_SET_DM_DIB);
	cam.setParamPixelClock(-1);
	cam.setParamExposure(5); // 50
	cam.setParamFramerate(fps); // 60
	cam.setParamGamma(2.2); // 2.2
	cam.setParamGain(150); // 100
	cam.setParamOffset(30); // 10

    cam.videoStart();
    cv::imshow("frame", frame);

    if(!writer.isOpened()) {
        THROW("Could not open '%s'\n", filename.c_str());
    }


#if TTY_AUTO_ENABLED
    fd = open(TTY_PORTNAME, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        THROW("error %d opening %s: %s", errno, TTY_PORTNAME, strerror (errno));
    }
    INFO("opened %s", TTY_PORTNAME);

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking
    //initscr();
    //noecho();
    //raw();
    //nodelay(stdscr, false);
    timeout(-1);
    usleep(200*1000);
    write(fd, all_on, strlen(all_on));
    usleep(200*1000);
    write(fd, all_on, strlen(all_on));
#endif


    TRY_CV(cv::namedWindow("frame", cv::WINDOW_NORMAL));

    while(alive) {
		if (!cam.videoFrame(frame, stats)) continue;
		if (frame.empty()) continue;
        //frame.copyTo(outimg);
        if (video) {
            writer.write(frame);
        }

        //char buf[256] = {0};
        //snprintf(buf, 256, "fps: %.2f [%ux%u]", stats.fps, frame.cols, frame.rows);
        //TRY_CV(cv::putText(frame, buf, cv::Point(0, frame.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0xFF, 0xFF, 0xFF, 0xFF)));

        //snprintf(buf, 256, "file '%s' %s", filename.c_str(), video ? "[RECORDING]" : "[PREVIEW]");
        //TRY_CV(cv::putText(frame, buf, cv::Point(0, frame.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0xFF, 0xFF, 0xFF, 0xFF)));

        cv::imshow("frame", frame);

        int key = cv::waitKey(1);
        if (key == 'q') alive = false;
        if (key == ' ') {
            video ^= true;
#if TTY_AUTO_ENABLED
            if(video) {
                write(fd, all_off, strlen(all_off));
                //usleep(400*1000);
                for(size_t i = 0; i < tty_auto_commands.size(); ++i) {
                    usleep(500000);
                    std::string cmd = tty_auto_commands[i];
                    INFO("sending: [%s]", cmd.c_str());
                    cmd += "\n";
                    write(fd, cmd.c_str(), cmd.size());
                }
            }
#endif
            printf("Recording: %s\n", video ? "true" : "false");
        }

    }


clean:
#if TTY_AUTO_ENABLED
    if(!(fd < 0)) {
        write(fd, "mode none\n", strlen("mode none\n"));
        close(fd);
    }
#endif
    cam.videoStop();
    return err;
error:
    ERR_CLEAN;
}


