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

bool AllPossiblePFromF(cv::Mat& F, cv::Mat& K, std::vector<arma::fmat>& Proj_camB);

bool DecomposeEtoRandT(cv::Mat& E, arma::fmat& R1, arma::fmat& R2, arma::fmat& t1, arma::fmat& t2);

bool checkRotationMatrix(arma::fmat& R);