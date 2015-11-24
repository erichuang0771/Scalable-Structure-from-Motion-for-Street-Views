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
struct seg
{
    image<rgb>* IMG;
    vector<DMatch> match;
    long ID;
    long match_ID;
    Mat img;
};

void blending(Mat &temp, Mat &temp2, Mat &result){
	Vec3b zeros;
	zeros[0] = 0;
	zeros[1] = 0;
	zeros[2] = 0;
	Vec3b x;
	  for (int i = 0; i < temp.cols; i++) {
        for (int ii = 0; ii < temp.rows; ii++) {
            if (temp.at < Vec3b > (ii, i) != zeros && temp2.at < Vec3b > (ii, i) != zeros) {
                x =  temp.at < Vec3b > (ii, i)*0.5 +  temp2.at < Vec3b > (ii, i)*0.5;
            } else if(temp.at < Vec3b > (ii, i) != zeros) {
                 x = temp.at < Vec3b > (ii, i);
            } else {
                 x = temp2.at < Vec3b > (ii, i);
            }
            result.at < Vec3b > (ii, i) = x;
        }
    }
	}

Mat stitching_two(match_seg mid, std::vector<seg> seg_A, std::vector<seg>  seg_B, Mat img_object, int seg_index, Mat result) {
    Mat Hl = findHomography(mid.obj, mid.mid, CV_RANSAC);
    Mat Hr = findHomography(mid.scene, mid.mid, CV_RANSAC);

    // Find the Homography Matrix
    // Use the Homography Matrix to warp the images

    std::cout << Hl;
    std::cout << "\n\n";
    std::cout << Hr;
    std::cout << "hehe\n\n";
    /*warpPerspective(seg_A[seg_index].img, result, Hl,
    		cv::Size(img_object.cols, img_object.rows));*/
    cv::Mat temp, temp2;
    warpPerspective(seg_B[seg_index].img, temp, Hr,
                    cv::Size(img_object.cols, img_object.rows));
    int threshold_value = 0;
    int max_BINARY_value = 256;
    Mat mask;

    //imshow("result",result);
    //waitKey();
    //exit(0);
    Mat result2;
   // temp.copyTo(result2, mask);

    warpPerspective(seg_A[seg_index].img, temp2, Hl,
                    cv::Size(img_object.cols, img_object.rows));

   // temp.copyTo(result2, mask);
    Mat haha;
	temp.copyTo(haha);
	blending(temp,temp2,haha);
	threshold(haha, mask, threshold_value, max_BINARY_value, THRESH_BINARY);
	haha.copyTo(result,mask);
    return result;
}

Mat stitching_bg(match_seg mid, Mat A, Mat B, Mat img_object) {
    Mat Hl = findHomography(mid.obj, mid.mid, CV_RANSAC);
    Mat Hr = findHomography(mid.scene, mid.mid, CV_RANSAC);

    // Find the Homography Matrix
    // Use the Homography Matrix to warp the images

    std::cout << Hl;
    std::cout << "\n\n";
    std::cout << Hr;
    std::cout << "hehe\n\n";
    /*warpPerspective(seg_A[seg_index].img, result, Hl,
    		cv::Size(img_object.cols, img_object.rows));*/
    cv::Mat temp, temp2;
    cv::Mat result;
    warpPerspective(B, temp, Hr,
                    cv::Size(img_object.cols, img_object.rows));
    int threshold_value = 0;
    int max_BINARY_value = 256;
    Mat mask, mask2;
    threshold(temp, mask, threshold_value, max_BINARY_value, THRESH_BINARY);

    temp.copyTo(result, mask);

    warpPerspective(A, temp2, Hl,
                    cv::Size(img_object.cols, img_object.rows));

    threshold(temp2, mask, threshold_value, max_BINARY_value, THRESH_BINARY);
    //temp2.copyTo(result, mask);
	blending(temp,temp2,result);

    return result;
}
