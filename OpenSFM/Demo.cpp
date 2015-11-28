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
#include "OpenSfM.h"




using namespace cv;
void parse_argv(int argc, char** argv) {
  if(argc != 2) {
    printf("Incorrect usage: Demo image_dir\n");
    exit(-1);
  }
}


int main(int argc, char** argv) {

  parse_argv(argc, argv);
  // int file_name_counter = atoi(argv[2]);

  // std::string render_dir = argv[1];
  
  // DIR* render_DIR;
  // if ((render_DIR = opendir(render_dir.c_str())) == NULL) {
  //   printf("Problem opening directory: %s\n", render_dir.c_str());
  //   exit(-1);
  // }
  //
   OpenSfM* tester = new OpenSfM();
   tester->loadParas(argv[1]);
/*
   string hehe = "heehhehe";
   tester->intrinsc_K << 1520.4 << 0      << 302.32 << arma::endr
                     << 0      << 1525.9 << 246.87 <<arma::endr
                     << 0      << 0      << 1      ;
   tester->run(argv[1]);*/
   return 0;
   


  // struct dirent* file;
  // while (file = readdir(render_DIR)) {



  //       }
  }
