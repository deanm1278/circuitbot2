/*
 * calibrate.cpp
 *
 *  Created on: Apr 6, 2016
 *      Author: deanm1278
 *
 * This class will search for a template in an image, and then send
 * motion data to calibrate the machine and correctly align with the template
 */

#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class calibrate {
public:
    calibrate(int camera, int board_height, int board_width);
    int find_error(float[2]);
    bool destroy_all_windows();
    virtual ~calibrate();
private:
    cv::Mat img;
    cv::VideoCapture *capture;
    cv::Size boardSize;
    std::vector<cv::Point2f> pointBuf;
};

#endif /* CALIBRATE_H_ */
