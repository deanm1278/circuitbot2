/*
 * calibrate.cpp
 *
 *  Created on: Apr 6, 2016
 *      Author: deanm1278
 *
 * This class will search for a template in an image, and then send
 * motion data to calibrate the machine and correctly align with the template
 */
#include <iostream>
#include <sstream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "calibrate.h"

using namespace cv;
using namespace std;

calibrate::calibrate(int camera, int board_height, int board_width){
	boardSize.height = board_height;
	boardSize.width = board_width;

    capture = new VideoCapture(camera);
    // set any  properties in the VideoCapture object
	capture->set(CV_CAP_PROP_FRAME_WIDTH,1280);   // width pixels
	capture->set(CV_CAP_PROP_FRAME_HEIGHT,720);   // height pixels
	capture->set(CV_CAP_PROP_FPS,1);
}

int calibrate::find_error(float[2]){
	 if(!capture->isOpened()){   // connect to the camera
			 cout << "Failed to connect to the camera." << endl;
			return 1;
	 }
	*capture >> img;          // populate the frame with captured image

	//find the corners on the chessboard
	bool found;
	found = findChessboardCorners( img, boardSize, pointBuf,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

	if (found) {
		/*

		 int cornersSize = pointBuf.size();

		 cout << cornersSize << " corners" << endl;

		 for(int k=0; k<cornersSize; k++){          //goes through all cv::Point2
		 f in the vector

		 float x = pointBuf[k].x;   //first value
		 float y = pointBuf[k].y;   //second value

		 cout << x << ", " << y << endl;

		 }

		 */

		line(img, pointBuf[0], pointBuf[boardSize.width - 1], Scalar(0, 255, 0), 2);

		line(img, pointBuf[0], pointBuf[pointBuf.size() - boardSize.width], Scalar(0,255,0), 2);

		line(img, pointBuf[pointBuf.size() - boardSize.width],pointBuf[pointBuf.size() - 1], Scalar(0, 255, 0), 2);

		line(img, pointBuf[pointBuf.size() - 1], pointBuf[boardSize.width - 1],Scalar(0, 255, 0), 2);

		// Draw the corners.
		//drawChessboardCorners( img, boardSize, Mat(pointBuf), found );
	} else {
		cout << "no chessboard found!" << endl;
	}

	imshow("Image View", img);

	return 0;
}

bool calibrate::destroy_all_windows(){
	destroyAllWindows();
	return true;
}

calibrate::~calibrate() {
	capture->release();
}
