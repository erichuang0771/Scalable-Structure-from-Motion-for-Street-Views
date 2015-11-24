	#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/stitching/stitcher.hpp"
#include <cstdio>
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "pnmfile.h"
#include "segment-image.h"

using namespace cv;

Mat cutting(Mat result, Mat target, Mat img_object){
	int threshold_value = 0;
	int max_BINARY_value = 256;
	Mat mask;
	threshold(target, mask, threshold_value, max_BINARY_value, THRESH_BINARY_INV);
	//bitwise_not(mask,mask);
	Mat image_masked = Mat::zeros(img_object.rows, img_object.cols, CV_8UC3);
	result.copyTo(image_masked, mask);
	//Size sz;Mat show;
	  //         pyrDown(image_masked, show, sz, BORDER_DEFAULT);
	    //       imshow("heheheh",show);waitKey();

	return image_masked;
}
