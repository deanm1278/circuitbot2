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

#include "opencv2/imgproc/imgproc.hpp"

class calibrate {
public:
    calibrate();
    int matchTemplate();
    virtual ~calibrate();
private:
    cv::Mat img; cv::Mat templ; cv::Mat result;
};

#endif /* CALIBRATE_H_ */