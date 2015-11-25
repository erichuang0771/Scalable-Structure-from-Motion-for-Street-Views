#include "OpenSfM.h"

using namespace std;
using namespace cv;

int OpenSfM::run(string dir_images){
	// dir should be dir of images, while for now, we just load two images
	// 
	cout<<"start to run OpenSfM........."<<endl<<endl;

	Mat imgA = imread(dir_images+"1.png", CV_LOAD_IMAGE_COLOR);
	Mat imgB = imread(dir_images+"2.png", CV_LOAD_IMAGE_COLOR);

	if(DEBUG) {
			cout<<"open img "<<dir_images+"1/2.png"<<endl;
			Mat H; hconcat(imgA,imgB,H);
			imshow("initial two view",H);
			waitKey(0);
	}


	last_frame* last_f = initalTwoViewRecon(imgA, imgB);


	return 0;
}



last_frame* OpenSfM::initalTwoViewRecon(cv::Mat& imA, cv::Mat& imB){
	
	/*  STEP 1
		Detect SIFT Feature
	 */
	SiftFeatureDetector detector;
	vector<KeyPoint> keypointA, keypointB;
	detector.detect(imA,keypointA);
	detector.detect(imB,keypointB);

	if(DEBUG) {
		/* code */
		 Mat img_keypoints_1; Mat img_keypoints_2;

 		 drawKeypoints( imA, keypointA, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  		 drawKeypoints( imB, keypointB, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

 		 //-- Show detected (drawn) keypoints
 		 imshow("Keypoints 1", img_keypoints_1 );
 		 imshow("Keypoints 2", img_keypoints_2 );

 		 waitKey(0);
	}

	/*  STEP 2
		Extract SIFT desc
	 */
	
	SiftDescriptorExtractor extractor;

	Mat descA, descB;
	extractor.compute(imA,keypointA,descA);
	extractor.compute(imB,keypointB,descB);


	/*-- Step 3: 
		Matching descriptor vectors using FLANN matcher
	*/

	 FlannBasedMatcher matcher;
	 vector< DMatch > matches;
	 matcher.match( descA, descB, matches );

	 double max_dist = 0; double min_dist = 100;

  	/* Setp 4 Good match
  	 Quick calculation of max and min distances between keypoints
  	 */
	  for( int i = 0; i < descA.rows; i++ ){
	  	 double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	  }

	 vector< DMatch > good_matches, test_matches;
	 vector< KeyPoint >P1, P2;
	 vector< Point2f > P1f, P2f;
	 int cnt = 0;
	  for( int i = 0; i < descA.rows; i++ ){
	   if( matches[i].distance <= max(2*min_dist, 0.02) ){
	   	 good_matches.push_back( matches[i]);
	   	 test_matches.push_back(DMatch(cnt,cnt,1.0)); cnt++;
	   	 P1.push_back(keypointA[matches[i].queryIdx]);
	   	 P2.push_back(keypointB[matches[i].trainIdx]);
	   	 P1f.push_back(keypointA[matches[i].queryIdx].pt);
	   	 P2f.push_back(keypointB[matches[i].trainIdx].pt);
	   	}
	  }
	  matches.clear();


	  if(DEBUG) {
	  	/* code */
	  	printf("-- Max dist : %f \n", max_dist );
  		printf("-- Min dist : %f \n", min_dist );
  		Mat img_matches;
  		drawMatches( imA, P1, imB, P2,
               test_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

 	 	//-- Show detected matches
  		imshow( "Good Matches", img_matches );

  		cout<<"size of desc A: "<<descA.rows<<" | "<<descA.cols<<endl;
  		cout<<"size of desc B: "<<descB.rows<<" | "<<descB.cols<<endl;


  		waitKey(0);

	  }
	  //P1.clear();
	  //P2.clear();
	  keypointA.clear();
	  keypointB.clear();



	  /* STEP 5  get F matrix
	  	
	   */
	  Mat fundamental_matrix = findFundamentalMat(P1f, P2f, FM_RANSAC, 3, 0.99);
	  
	  if(DEBUG) {
	  	/* code */
	  	Mat Epilines;
	  	computeCorrespondEpilines(Mat(P1f),1,fundamental_matrix,Epilines);
	  	cout<<"size of Epilines: "<<Epilines.rows<<" | "<<Epilines.cols<<endl;
	  	cout<<"depth of Epilines: "<<Epilines.depth()<<endl;

	  	Mat Epilines_show = imA;
	 	for(int i = 0; i < Epilines.rows; i++){
	 		float x = P1f[i].x;
	 		float y = P1f[i].y;
	 		// line(Epilines_show,Point(x,(-Epilines.at<float>(i,2)-(x)*Epilines.at<float>(i,0))/Epilines.at<float>(i,1)),
	 		// 					Point(x+100,(-Epilines.at<float>(i,2)-(x+1)*Epilines.at<float>(i,0))/Epilines.at<float>(i,1)),
	 		// 					CV_RGB(128, 128, 0));
	 		cout<<"point: "<<x<<"  "<<y<<endl;
	 	}
	 	Mat Epilines_show_pts;
	 	drawKeypoints( Epilines_show, P1, Epilines_show_pts, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	 	imshow("Epilines_show",Epilines_show_pts);
	 	waitKey(0);

	  }

	 // mat arma_mat( reinterpret_cast<double*>opencv_mat.data, opencv_mat.rows, opencv_mat.cols )




	return new last_frame;


}
