#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <cstdio>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <cstdlib>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <stdarg.h>
#include <cv.h>
#include <highgui.h>
#include <armadillo>
#include <numeric>

#define DEBUG 1

struct last_frame{
			std::vector<cv::KeyPoint> features;
			cv::Mat decs;
			arma::fmat length;
};


class OpenSfM{

public:
	arma::fmat intrinsc_K;
	int min_dist;
	std::vector<cv::Mat> images;

	cv::Mat* featureTable;
	std::vector<arma::fmat*>* camProjTable;
	std::vector<arma::umat*>* featureCell; //two dim vecctor
	std::vector<arma::fmat*>* cameraPose;
	//Z sparse matrix, when final triangulation, conver to arma & using find to speed up
	std::vector<unsigned>* Z_i;
	std::vector<unsigned>* Z_j;
	std::vector<unsigned>* Z_v;

bool loadParas(std::string dir);
/* Yilin Yang
	load intrinsc K and all thresholds....from configure txt file
	input: unsure
	output: 1 if succcess
		   0 if error
 */

int run();
/* run structure from motion
	input: read from string
	output: 1 success and 0 fails
 */

private:

int multiViewTriangulation(arma::umat& index , cv::Mat& ims);
/* Yilin Yang
	update 3d points in feature table that viewed by at least three camseras
	input: index, the entry that you want to update
			ims: image, in order to get color info
	ouput: 1 if succcess
		   0 if error
 */
last_frame* updateStruture(cv::Mat& ims, last_frame* last_frame,cv::Mat& debug_im );
/*  Eric Huang & Yilin Yang
	write new data into all tables and compute camera R and T
	input: ims: new image that going to estimates
		   last_frame: the struct that contains all info needed in updating
	ouput: the last_frame struct

 */
last_frame* initalTwoViewRecon(cv::Mat& imA, cv::Mat& imB);
/*  Eric Huang
	init all internal tables and create two view gemotry
	input: imA imB: new image that going to estimates

	ouput: last_frame: the struct that contains all info needed in updating

 */

};
