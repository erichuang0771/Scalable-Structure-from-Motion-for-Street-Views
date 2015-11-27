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
	  	and check Epolier line
	   */
	  Mat mask;
	  Mat fundamental_matrix = findFundamentalMat(P1f, P2f, FM_RANSAC, 3, 0.99, mask);
	  
	  if(DEBUG) {
	  	/* code */
	  	Mat Epilines;
	  	computeCorrespondEpilines(Mat(P1f),1,fundamental_matrix,Epilines);
	  	cout<<"size of Epilines: "<<Epilines.rows<<" | "<<Epilines.cols<<endl;
	  	cout<<"depth of Epilines: "<<Epilines.depth()<<endl;

	  	//float top_horizontal[3] =    {0, 1, 0};
	  	 Point3f top_horizontal = Point3f(0,1,0);
		 Point3f left_vertical  =   Point3f(1, 0, 0); 
		 Point3f bottom_horizontal = Point3f(0, 1, -imA.rows); 
		 Point3f right_vertical =    Point3f(1, 0, -imA.cols); 

	  	Mat Epilines_show = imA;
	 	for(int i = 0; i < Epilines.rows; i++){
	 		Point2f A;
	 		Point2f B;

	 		Point3f Eline = Point3f(Epilines.at<float>(i,0),Epilines.at<float>(i,1),Epilines.at<float>(i,2));
	 		Point3f candidate_1 = top_horizontal.cross(Eline);
	 		Point2f candidate_1_cord = Point2f(candidate_1.x/candidate_1.z, candidate_1.y/candidate_1.z);
	 		if(candidate_1_cord.x >= 0 && candidate_1_cord.x <= imA.cols) {
	 			/* code */
	 			A = candidate_1_cord;
	 		}
	 		Point3f candidate_2 = left_vertical.cross(Eline);
	 		Point2f candidate_2_cord = Point2f(candidate_2.x/candidate_2.z, candidate_2.y/candidate_2.z);
	 		if(candidate_2_cord.y >= 0 && candidate_2_cord.y <= imA.cols) {
	 			/* code */
	 			A = candidate_2_cord;
	 		}
	 		Point3f candidate_3 = bottom_horizontal.cross(Eline);
	 		Point2f candidate_3_cord = Point2f(candidate_3.x/candidate_3.z, candidate_3.y/candidate_3.z);
	 		if(candidate_3_cord.x >= 0 && candidate_3_cord.x <= imA.cols) {
	 			/* code */
	 			B = candidate_3_cord;
	 		}
	 		Point3f candidate_4 = right_vertical.cross(Eline);
	 		Point2f candidate_4_cord = Point2f(candidate_4.x/candidate_4.z, candidate_4.y/candidate_4.z);
	 		if(candidate_4_cord.y >= 0 && candidate_4_cord.y <= imA.cols) {
	 			/* code */
	 			B = candidate_4_cord;
	 		}

	 		line(Epilines_show,A,B,Scalar(0,255,0));
	 		// cout<<"point: "<<x<<"  "<<y<<endl;
	 	}
	 	Mat Epilines_show_pts;
	 	drawKeypoints( Epilines_show, P1, Epilines_show_pts, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	 	imshow("Epilines_show",Epilines_show_pts);
	 	waitKey(0);
	  }

	  /* STEP 6  
	  	create tables
	   */
	  int NUM = P1f.size();
	  //handle featureTable;
	  this->featureTable = new Mat(NUM,128+3+3,CV_32FC1);
	  this->


	  this->camProjTable = new vector<arma::fmat*>();

	 // camProjTable->push_back(new fmat);
	  
	  

	 // mat arma_mat( reinterpret_cast<double*>opencv_mat.data, opencv_mat.rows, opencv_mat.cols )

	  


	return new last_frame;


}
