/*
 * calibrate.cpp
 *
 *  Created on: Apr 6, 2016
 *      Author: deanm1278
 *
 * This class will search for a template in an image, and then send
 * motion data to calibrate the machine and correctly align with the template
 */

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include "calibrate.h"

using namespace std;
using namespace cv;

int calibrate::matchTemplate(){
    /// Load image and template
    img = imread( "haystack.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    templ = imread( "needle.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    
    if(img.empty() || templ.empty())
        return -1;
    
    int match_method = CV_TM_SQDIFF;
    
    /// Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    result.create( result_rows, result_cols, CV_32FC1 );

    /// Do the Matching and Normalize
    cv::matchTemplate( img, templ, result, match_method );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
      { matchLoc = minLoc; }
    else
      { matchLoc = maxLoc; }

    cout << matchLoc.x + templ.cols << "," << matchLoc.y + templ.rows << endl;
}

calibrate::calibrate(){
    
}

calibrate::~calibrate() {
	// TODO Auto-generated destructor stub
}