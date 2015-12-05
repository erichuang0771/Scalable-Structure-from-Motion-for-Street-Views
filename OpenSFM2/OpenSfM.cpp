#include "OpenSfM.h"
#include "GeometryFunctions.h"
#include "BA.h"

using namespace std;
using namespace cv;

int OpenSfM::run(){
	// dir should be dir of images, while for now, we just load two images
	cout<<"start to run OpenSfM........."<<endl<<endl;

	Mat imgA = this->images[0];
	Mat imgB = this->images[1];
	if( !imgA.data || !imgB.data){
		cerr<<"no data!\n"<<endl;
	}
	if(DEBUG) {
			Mat H; hconcat(imgA,imgB,H);
			imshow("initial two view",H);
			waitKey(0);
	}
	last_frame* last_f = initalTwoViewRecon(imgA, imgB);
for (size_t i = 2; i < 5; i++) {
	/* code */
	last_frame* next_f = updateStruture(images[i],last_f, images[i-1]);
	last_f = next_f;
}

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

	if(!DEBUG) {
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
		 //*here used to be descA.rows
	  for( int i = 0; i < matches.size(); i++ ){
	  	 double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	  }

	 vector< DMatch > good_matches, test_matches;
	 vector< KeyPoint >P1, P2;
	 vector< Point2f > P1f, P2f;

	 int cnt = 0;
	 //*here used to be descA.rows
	  for( int i = 0; i < matches.size(); i++ ){
	   if( matches[i].distance <= max(this->min_dist*min_dist, 0.02) ){
	   	 good_matches.push_back( matches[i]);
	   	 test_matches.push_back(DMatch(cnt,cnt,1.0)); cnt++;
	   	 P1.push_back(keypointA[matches[i].queryIdx]);
	   	 P2.push_back(keypointB[matches[i].trainIdx]);
	   	 P1f.push_back(keypointA[matches[i].queryIdx].pt);
	   	 P2f.push_back(keypointB[matches[i].trainIdx].pt);
	   	}
	  }
		Mat descB_candidate(good_matches.size(),128,descB.depth());
		for (size_t i = 0; i < good_matches.size(); i++) {
			/* prepare descB_candidate for last_frame */
		 descB.row(good_matches[i].trainIdx).copyTo(descB_candidate.row(i));
		}

	  matches.clear();
		printf("-- Max dist : %f \n", max_dist );
		printf("-- Min dist : %f \n", min_dist );

	  if(!DEBUG) {
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
  		cout<<"size of good matches: "<<good_matches.size()<<endl;
  		waitKey(0);
	  }
	  P1.clear();
	  P2.clear();
	  keypointA.clear();
	  keypointB.clear();


	  /* STEP 5  get F matrix
	  	and check Epolier line
	   */
	  Mat mask;
	  std::vector<Point2f> P1f_norm = P1f, P2f_norm = P2f;
		//std::cout << "rowcol" << this -> img_rows << "|" << this -> img_cols << std::endl;
		for(int i = 0; i < P1f.size(); i++){
			P1f_norm[i].x = P1f_norm[i].x / (this -> img_cols);
			P1f_norm[i].y = P1f_norm[i].y / (this -> img_rows);
			P2f_norm[i].x = P2f_norm[i].x / (this -> img_cols);
			P2f_norm[i].y = P2f_norm[i].y / (this -> img_rows);
		}
		Mat scale = Mat::zeros(3, 3, CV_64FC1);
		scale.at<double>(0, 0) = 1.0 / (this -> img_cols);
		scale.at<double>(1, 1) = 1.0 / (this -> img_rows);
		scale.at<double>(2, 2) = 1.0;
	  Mat fundamental_matrix = findFundamentalMat(P1f_norm, P2f_norm, FM_RANSAC, 0.001, 0.99, mask);
		fundamental_matrix = scale * fundamental_matrix * scale;
		//arma::fmat hehehe; hehehe.load("FFF.mat",arma::raw_ascii);
		//Mat fundamental_matrix( 3, 3, CV_32FC1, hehehe.memptr() );
		//fundamental_matrix.convertTo(fundamental_matrix,CV_64FC1);
		//fundamental_matrix = fundamental_matrix.t();
		cout<<"**********************\n"<<fundamental_matrix<<"*********************\n";


		if (!DEBUG) {
			/* save fundamental_matrix */
			arma::mat F_saved(reinterpret_cast<double*>(fundamental_matrix.data),fundamental_matrix.rows,fundamental_matrix.cols);
			F_saved = F_saved.t();
			F_saved.save("fundamental_matrix_from_p1f_p2f.mat", arma::raw_ascii);
		}

	  if(DEBUG) {
	  	/* draw epolir line */
	  	Mat Epilines;
	  	computeCorrespondEpilines(Mat(P1f),1,fundamental_matrix,Epilines);
	  	// cout<<"size of Epilines: "<<Epilines.rows<<" | "<<Epilines.cols<<endl;
	  	// cout<<"depth of Epilines: "<<Epilines.depth()<<endl;

	   Point3f top_horizontal = Point3f(0,1,0);
		 Point3f left_vertical  =   Point3f(1, 0, 0);
		 Point3f bottom_horizontal = Point3f(0, 1, -imA.rows);
		 Point3f right_vertical =    Point3f(1, 0, -imA.cols);

	  	Mat Epilines_show;
			imB.copyTo(Epilines_show);
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
	 	drawKeypoints( Epilines_show, P2, Epilines_show_pts, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	 	imshow("Epilines_show",Epilines_show_pts);
	 	waitKey(0);
	  }

	  /* STEP 6
	  	create tables
	   */
	  int NUM = 0;
	  vector< Point2f > P1_inliers, P2_inliers;
		std::vector<DMatch> test_inliear_match;
		std::vector<KeyPoint> P1in,P2in;
		int ccc = 0;
	  for(unsigned i = 0; i < mask.rows; ++i) {
	  	/* extract inliears */
	  	if(mask.at<char>(i,0) != 0){ NUM++;
	  		P1_inliers.push_back(P1f[i]);
	  		P2_inliers.push_back(P2f[i]);
				P1in.push_back(KeyPoint(P1f[i],1.0));
				P2in.push_back(KeyPoint(P2f[i],1.0));
				test_inliear_match.push_back(DMatch(ccc,ccc,1.0));ccc++;
	  	}
	  }

		if(DEBUG){
			/* draw inliear matches*/
			Mat inliears_match;
			drawMatches( imA, P1in, imB, P2in,
							 test_inliear_match, inliears_match, Scalar::all(-1), Scalar::all(-1),
							 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
							 imshow("inilear_matches", inliears_match);waitKey(0);
		}

	  //handle featureTable;
	  int num_of_feature = P1f.size();
	  this->featureTable = new Mat(num_of_feature,128+3+3,CV_32FC1);
	  this->featureCell = new vector<arma::fmat*>();
	  this->cameraPose = new vector<arma::fmat*>();
	  this->camProjTable = new vector<arma::fmat*>();
	  this->Z_i = new vector<unsigned>(num_of_feature*2);
		std::cout << "num_of_feature: "<< num_of_feature << std::endl;
		for (size_t i = 0; i < num_of_feature; i++) (*Z_i)[i] = 0;
		for (size_t i = num_of_feature; i < num_of_feature*2; i++) (*Z_i)[i] = 1;

	  this->Z_j = new vector<unsigned>(num_of_feature*2);
		for (size_t i = 0; i < num_of_feature; i++) (*Z_j)[i] = i;
		for (size_t i = num_of_feature; i < num_of_feature*2; i++) (*Z_j)[i] = i-num_of_feature;

	  this->Z_v = new vector<char>(num_of_feature*2,1);
		for (size_t i = 0; i < num_of_feature*2; i++) (*Z_v)[i] = 1;


	  if(DEBUG){
			cout<<"depth of mask: "<<mask.depth()<<endl;
			cout<<"# of inliners: "<<NUM<<endl;
			cout<<"# of all_features: "<<num_of_feature<<endl;
			cout<<"size of good matches: "<<good_matches.size()<<endl;
		}

	  for(unsigned i = 0; i < num_of_feature; ++i) {
			/* insert tabel */
	  		//1.copy desc of camB to the table for future match
	  		descB.row(good_matches[i].trainIdx).copyTo(((this->featureTable)->row(i)).colRange(0,128));
	  		//if(DEBUG) cout<<(this->featureTable)->row(i).colRange(0,2)<<" | "<<descB.row(good_matches[i].trainIdx).colRange(0,2)<<endl;

	  		//2.insert correspounding 2D points to feature cell
	  		arma::fmat* tmp_feature_per_cell = new arma::fmat(2,2);
	  		*tmp_feature_per_cell << P1f[i].x << P1f[i].y << arma::endr << P2f[i].x << P2f[i].y << arma::endr;
	  		featureCell->push_back(tmp_feature_per_cell);
		}
		//3. init Z table;
		//done when initilized

		//4. init camProjTable
		arma::fmat* tmp_camProjTable = new arma::fmat(3,4);
		*tmp_camProjTable  << 1.0 << 0.0 << 0.0 << 0.0 << arma::endr
						   			   << 0.0 << 1.0 << 0.0 << 0.0<< arma::endr
						   				 << 0.0 << 0.0 << 1.0 << 0.0;
		// *tmp_camProjTable  << 0.0219 << 0.9833 << -0.1807 << -0.0292 << arma::endr
		// 				   << 0.9986 << -0.0127 << 0.0520 << -0.0242 << arma::endr
		// 				   << 0.0488 << -0.1816 << -0.9822 << 0.5227;
		arma::fmat* tmp_camPoseTable = new arma::fmat(3,4);
		//copy [I 0] from Proj_1
		*tmp_camPoseTable = *tmp_camProjTable;

		this->cameraPose->push_back(tmp_camPoseTable);

		*tmp_camProjTable = (this->intrinsc_K)*(*tmp_camProjTable);
		this->camProjTable->push_back(tmp_camProjTable);

		// solve projCam for the camB
		Mat K = Mat(3,3,CV_32FC1,(this->intrinsc_K).memptr());
		K.convertTo(K,CV_64FC1);
		K = K.t();
		// if(DEBUG) cout<<" intrinsc_K : "<<K<<endl;
		// if(DEBUG) cout<<" FFF : "<<fundamental_matrix<<endl;

		vector<arma::fmat> Projs_camB;
		AllPossiblePFromF(fundamental_matrix, K, Projs_camB);
		arma::fmat cam2(3,4);

		std::vector<arma::fmat> poseB;
		for(unsigned i = 0; i < Projs_camB.size(); ++i) {
			// pose*K to become Proj_B
			poseB.push_back(Projs_camB[i]);
			Projs_camB[i] = (this->intrinsc_K)*Projs_camB[i];
		}

		Mat camP_A( 4, 3, CV_32FC1, (*camProjTable)[0]->memptr() ); camP_A = camP_A.t();
		vector<Mat> point4D(4);
			K.convertTo(K,CV_32FC1);
			int max_correct_pts = 0; int best_index = -1;
			if(!DEBUG) (*camProjTable)[0]->save("A_proj.mat",arma::raw_ascii);

		for(unsigned i = 0; i < Projs_camB.size(); ++i){
			/* test each Projs_camB */
			int counter_correct_pts = 0;
			Mat camP_B( 4, 3, CV_32FC1, Projs_camB[i].memptr() ); camP_B = camP_B.t();

			if (!DEBUG) {
				/* code */
				Projs_camB[i].save(to_string(i)+"proj.mat",arma::raw_ascii);

				arma::fmat savedA = arma::fmat(P1f.size(),2);
				arma::fmat savedB = arma::fmat(P2f.size(),2);
				for (int i = 0; i < P1f.size(); i++) {
					/* code */
					savedA(i,0) = P1f[i].x;
					savedA(i,1) = P1f[i].y;
					savedB(i,0) = P2f[i].x;
					savedB(i,1) = P2f[i].y;
				}
				savedA.save("P1f.mat",arma::raw_ascii);
				savedB.save("P2f.mat",arma::raw_ascii);

			}
			std::cout << "CamA: "<< camP_A << std::endl;
			std::cout << "CamB: "<< camP_B << std::endl;

			triangulatePoints(camP_A,camP_B,P1_inliers,P2_inliers,point4D[i]);

			arma::fmat point4Dsaved(reinterpret_cast<float*>(point4D[i].data), point4D[i].cols, point4D[i].rows );
			if(!DEBUG) point4Dsaved.save(to_string(i)+"point4Dsaved.mat",arma::raw_ascii);

			convertPointsFromHomogeneous(point4D[i].t(),point4D[i]); // confirm it works and correct
			cout<<"size of point4D:"<<endl<<point4D[i].rows<<endl<<point4D[i].cols<<endl;

			Mat tmo(point4D[i].rows,3,CV_32FC1);
			for(unsigned x = 0; x < point4D[i].rows; ++x) {
				/* fix openCV bug */
				 tmo.at<float>(x,0) = point4D[i].at<Vec3f>(x,0)[0] ;
				 tmo.at<float>(x,1) = point4D[i].at<Vec3f>(x,0)[1] ;
				 tmo.at<float>(x,2) = point4D[i].at<Vec3f>(x,0)[2] ;
				if(point4D[i].at<float>(x,2) > 0) counter_correct_pts++;
			}
			Mat one = Mat::ones(tmo.rows,1,CV_32FC1);
			Mat tmp_A2B;
			tmo.copyTo(point4D[i]);
			hconcat(tmo,one,tmp_A2B);

			Mat point4D_in_camB = K.inv()*camP_B*tmp_A2B.t();
			point4D_in_camB = point4D_in_camB.t();

			for(unsigned x = 0; x < point4D_in_camB.rows; ++x) {
				/* code */
				if(point4D_in_camB.at<float>(x,2) > 0) counter_correct_pts++;
			}
			cout<<"counter_correct_pts: "<<counter_correct_pts<<" / "<< point4D[i].rows*2<<endl;
			if(counter_correct_pts > max_correct_pts){
				best_index = i;
				max_correct_pts = counter_correct_pts;
			}
		}


		cout<<"best index: "<<best_index<<endl;
		arma::fmat *tmp_camProjTable_B = new arma::fmat(3,4); *tmp_camProjTable_B = Projs_camB[best_index];
		/*
		push the second camProjection
		*/

		this->camProjTable->push_back(tmp_camProjTable_B);

	  	//handle poseTable

	  	 arma::fmat * pose_B = new arma::fmat(3,4);
	  	 *pose_B = poseB[best_index];
			 if(DEBUG){
				 cout<<"pose_B: "<<poseB[best_index]<<endl;
			 }
			 /*
		 	push the second camPose
		 	*/
	  	  this->cameraPose->push_back(pose_B);

	// 			/*
	// 				prepare BA
	// 			*/
	//
	//  arma::fmat point2DA( P1_inliers.size() , 2 );
	//  arma::fmat point2DB( P1_inliers.size() , 2 );
	//
	//  for(unsigned i = 0; i < P1_inliers.size(); ++i) {
	//  	/* code */
	//  	point2DA.at(i,0) = P1_inliers[i].x;
	//  	point2DA.at(i,1) = P1_inliers[i].y;
	//  	point2DB.at(i,0) = P2_inliers[i].x;
	//  	point2DB.at(i,1) = P2_inliers[i].y;
	// }
	//
	  arma::fmat point4D_( reinterpret_cast<float*>(point4D[best_index].data), point4D[best_index].cols, point4D[best_index].rows );
	  point4D_ = point4D_.t();
	//  point4D_.save("openCV_final_pts.mat",arma::raw_ascii);
	//  cout<<"ready BA: "<<point2DA.n_rows<<point2DB.n_rows<<point4D_.n_rows<<endl;
	// //  cout<<"pose: "<<*(*cameraPose)[0]<<"\n pose: "<<*(*cameraPose)[1]<<endl;
	//
	//
	// std::cout << "bundle_adjustment start!" << std::endl;
	//  local_bundle_adjustment( intrinsc_K,
  //                           *(*cameraPose)[0],
  //                           *(*cameraPose)[1],
  //                           point2DA,
  //                           point2DB,
  //                           point4D_);
	// 	std::cout << "bundle_adjustment done!" << std::endl;
	/*
			handle last frame
	*/
	last_frame* last_f = new last_frame;
	last_f->features = P1in;
	last_f->decs = Mat(P1_inliers.size(),128,descB_candidate.depth());
	cout<<"handle length!\n";
	int ccnt = 0;
	for (size_t i = 0; i < mask.rows; i++) {
		/* copy final desc */
			if(mask.at<char>(i,0) != 0){
			 	descB_candidate.row(i).copyTo((last_f->decs).row(ccnt)); ccnt++;
			}
	}
	// copy pts3D for next PnP
	point4D[best_index].copyTo(last_f->pts3D);

	last_f->length = arma::fmat(P1_inliers.size(),1);
	for (size_t i = 0; i < P1_inliers.size(); i++) {
		/* compute length */
		(last_f->length).row(i) = sqrt(point4D_(i,0)*point4D_(i,0) + point4D_(i,1)*point4D_(i,1) + point4D_(i,2)*point4D_(i,2));
	}
	/*
			frame 1 triangulation
	*/
	arma::umat index_B(num_of_feature,1);
	for (size_t i = 0; i < num_of_feature; i++) {
				index_B(i,0) = i;
	}

	multiViewTriangulation(index_B , imB);
	arma::fmat featureTable_arma(reinterpret_cast<float*>((*(this->featureTable)).data), 128+6, (*(this->featureTable)).rows);
	featureTable_arma.save("featureTable_0.mat",arma::raw_ascii);

	cout<<"initalTwoViewRecon done\n";
	if(!DEBUG){
		// cout<<"last_f->features: \n"<<last_f->features<<endl;
		// cout<<"last_f->decs: \n"<<last_f->decs<<endl;
		// cout<<"last_f->length: \n"<<last_f->length<<endl;
	}
	return last_f;
	}







last_frame* OpenSfM::updateStruture(cv::Mat& imC, last_frame* last_f, cv::Mat& debug_im ){
		std::cout << "\n\n\n\nconnected updateStruture!..." << std::endl;
		/*  STEP 1
			Detect SIFT Feature
		 */
		SiftFeatureDetector detector;
	 	vector<KeyPoint> keypointC;
	 	detector.detect(imC,keypointC);
		/*  STEP 2
			Extract SIFT desc
		 */
		 SiftDescriptorExtractor extractor;
		 Mat descC;
		 extractor.compute(imC,keypointC,descC);
		 /*-- Step 3:
	 		Matching descriptor vectors using FLANN matcher
	 	*/
		FlannBasedMatcher matcher;
		vector< DMatch > matches;
		matcher.match( last_f->decs, descC, matches );
		/* Setp 4 Good match
		 Quick calculation of max and min distances between keypoints
		 */
		 cout<<"match size: "<<matches.size()<<endl;
		 cout<<"descC size: "<<descC.rows<<endl;
		 cout<<"last_f->features size: "<< (last_f->features).size()<<endl;
		 cout<<"last_f->decs size: "<<(last_f->decs).rows<<endl;
		double max_dist = 0; double min_dist = 100;
		for( int i = 0; i < matches.size(); i++ ){
			 double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}

		vector< DMatch > good_matches, test_matches;
	  vector< KeyPoint >P1, P2;
		vector< Point2f >  P2f;

		int cnt = 0;
		Mat matched_3D_pts(0,3,CV_32FC1);
		 for( int i = 0; i < matches.size(); i++ ){
			if( matches[i].distance <= max(5*min_dist, 0.02)){
				good_matches.push_back(matches[i]);
				 test_matches.push_back(DMatch(cnt,cnt,1.0)); cnt++;
				// std::cout << "/* queryIdx */ " << (last_f->features)[matches[i].queryIdx].pt << std::endl;
				  P1.push_back( (last_f->features)[matches[i].queryIdx] );
				  P2.push_back(keypointC[matches[i].trainIdx]);
				//  P1f.push_back((last_f->features)[matches[i].queryIdx].pt);
				  P2f.push_back(keypointC[matches[i].trainIdx].pt);
				 matched_3D_pts.push_back((last_f->pts3D).row(i));
				 //prepare maatched 3D pts
			 }
		 }
		 std::cout << "done!!!!" << std::endl;
		Mat descC_candidate(good_matches.size(),128,descC.depth());
	 	for (size_t i = 0; i < good_matches.size(); i++) {
	 		/* prepare descB_candidate for last_frame */
	 	 descC.row(good_matches[i].trainIdx).copyTo(descC_candidate.row(i));
	 	}
		matches.clear();
		//draw good match!
		if(DEBUG) {
	  	/* code */
	  	printf("-- Max dist : %f \n", max_dist );
  		printf("-- Min dist : %f \n", min_dist );
  		Mat img_matches;
  		drawMatches( debug_im, P1, imC, P2,
               test_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
 	 	//-- Show detected matches
  		imshow( "Good Matches C", img_matches );
  		cout<<"size of desc A: "<<(last_f->decs).rows<<" | "<<(last_f->decs).cols<<endl;
  		cout<<"size of desc B: "<<descC.rows<<" | "<<descC.cols<<endl;
  		cout<<"size of good matches: "<<good_matches.size()<<endl;
  		waitKey(0);
	  }
	  P1.clear();
	  P2.clear();
	  keypointC.clear();
		/*
		Solve PnP
		*/
		//transfer intrinsc_K from arma to Mat
		std::cout << "solve PnP...." << std::endl;
		std::cout << "matched_3D_pts.rows: "<< matched_3D_pts.rows << std::endl;
		std::cout << "P2f.size: "<< P2f.size() << std::endl;

		Mat Mat_K(3,3,CV_32FC1, (this->intrinsc_K).memptr());
		Mat K_tmp;
		Mat_K.copyTo(K_tmp);
		std::cout << "K_tmp: " << K_tmp <<endl;

		K_tmp = K_tmp.t();
		Mat rvec, tvec;
		if (DEBUG) {
			/* code */
			//  std::cout << matched_3D_pts << std::endl;
			//  std::cout << Mat_K << std::endl;
			// for (size_t i = 0; i < P1f.size(); i++) {
			// 	/* code */
			// 	std::cout << "P1f: "<< P1f[i] << std::endl;
			// }
		}
		std::vector<float> v;
		std::vector<int> PnP_inliners;
		solvePnPRansac(matched_3D_pts, P2f, K_tmp, v, rvec, tvec, false,100,3.0,100,PnP_inliners);
		std::cout << "PnP_inliners size: "<< PnP_inliners.size() << std::endl;

		Mat rot;
		Rodrigues(rvec,rot);
		Mat Pose_C(3,4,CV_64FC1); hconcat(rot,tvec,Pose_C);
		K_tmp.convertTo(K_tmp,CV_64FC1);
		arma::mat new_pose_C(reinterpret_cast<double*>(Pose_C.data), 4, 3);
		new_pose_C = new_pose_C.t();
		arma::fmat* new_pose_C_f = new arma::fmat(3,4);
		*new_pose_C_f = arma::conv_to<arma::fmat>::from(new_pose_C);
		(this->cameraPose)->push_back(new_pose_C_f);
		cout<<"cameraPose: "<<Pose_C<<endl;

		//  std::cout << "Pose_C.depth(): "<< Pose_C.depth() << std::endl;
		// cout<<"mat_K: "<<Mat_K<<endl;
		Mat Proj_C; Proj_C = K_tmp*Pose_C;
		std::cout << "rot: "<< rot << std::endl;
		std::cout << "tvec: "<< tvec << std::endl;
		 arma::mat new_proj_C(reinterpret_cast<double*>(Proj_C.data), 4, 3);
		 new_proj_C = new_proj_C.t();
		 arma::fmat* new_proj_C_f = new arma::fmat(3,4);
		 *new_proj_C_f = arma::conv_to<arma::fmat>::from(new_proj_C);
		 (this->camProjTable)->push_back(new_proj_C_f);
		 std::cout << "intrinsc_K: " << (this->intrinsc_K) <<endl;
		 std::cout << "PnP: solved proj: "<< Proj_C << std::endl;
		 std::cout << "PnP: invsoled pose: "<< K_tmp .inv()*Proj_C << std::endl;

		 /*
		 Bundle Adustment Starts
		 */
	// 	 /*
	// 	 prepare BA:
	// 	 */
	// 	 std::cout << "BA check size: "<< PnP_inliners.size() << std::endl;
	// 	 arma::fmat point4D_tmp( reinterpret_cast<float*>((matched_3D_pts).data), (matched_3D_pts).cols, (matched_3D_pts).rows );
	// 	 point4D_tmp = point4D_tmp.t();
	// 	 arma::fmat point4D_(PnP_inliners.size(),3);
	// 	 for (size_t i = 0; i < PnP_inliners.size(); i++) {
	// 		 /* code */
	// 			 point4D_.row(i) = point4D_tmp.row(PnP_inliners[i]);
	// 	 }
	 //
	 //
	// 	arma::fmat point2DA( PnP_inliners.size() , 2 );
	// 	arma::fmat point2DB( PnP_inliners.size() , 2 );
	//  //  std::cout << "" << std::endl;
	// 	for(unsigned i = 0; i < PnP_inliners.size(); ++i) {
	// 	 /* code */
	// 	 point2DA.at(i,0) = P1[ PnP_inliners[i] ].pt.x;
	// 	 point2DA.at(i,1) = P1[ PnP_inliners[i] ].pt.y;
	// 	 point2DB.at(i,0) = P2[ PnP_inliners[i] ].pt.x;
	// 	 point2DB.at(i,1) = P2[ PnP_inliners[i] ].pt.y;
	//  }
	// 	 int camNum = (*cameraPose).size();
	// 		std::cout << "bundle_adjustment start!" << std::endl;
	// 		std::cout << "intrinsc_K BA: " << intrinsc_K <<  std::endl;
	// 		 local_bundle_adjustment( intrinsc_K,
	// 															 *(*cameraPose)[camNum-2],
	// 															 *(*cameraPose)[camNum-1],
	// 															 point2DA,
	// 															 point2DB,
	// 															 point4D_);
	// 			std::cout << "bundle_adjustment done!" << std::endl;
				/*
			 Bundle Adustment Ends
			 */


		//  new_proj_C_f->save("camProjM"+to_string(i)+".mat",arma::raw_ascii);

		/*UPDATE
		Projection & Pose table
		*/
		// arma::mat* new_pose_C = new arma::fmat(reinterpret_cast<double*>(Pose_C.data), 4, 3);
		// *new_pose_C = new_pose_C->t();
		// (this->camProjTable)->push_back(new_pose_C);

		/* IMPORTANT
			IF CAMPOSE SOLVED BY PNP IS NOT CORRECT OR WE CAN ALSO IMPLEMENT CAMERA POSE
			FROM F MATRIX, THEN COMPARE RESULTS.
		*/

		/*
		fill in big table
		*/

		matcher.clear();
		vector< DMatch > all_matches;
		std::cout << "descC size(): "<< descC.rows<< " | "<<descC.cols << std::endl;
		Mat subview;
		((this->featureTable)->colRange(0,128)).copyTo(subview);
		std::cout << "(this->featureTable)->colRange(0,128) size(): "<< subview.rows<< " | "<<subview.cols << std::endl;
		/*	IMPORTANT
			using small descriptor to match big table, and we select the appropriate ones
		*/
		matcher.match(  descC, subview , all_matches );
		subview.release();
		std::cout << "init all_matches_C size()"<< all_matches.size() << std::endl;

		double all_max_dist = 0; double all_min_dist = 100;
		for( int i = 0; i < all_matches.size(); i++ ){
			 double dist = all_matches[i].distance;
			if( dist < all_min_dist ) all_min_dist = dist;
			if( dist > all_max_dist ) all_max_dist = dist;
		}

		vector< DMatch > all_good_matches, all_test_matches;
		vector< KeyPoint > all_PC, all_last;
		// vector< Point2f > all_P1f, all_P2f;
		printf("-- Max dist : %f \n", all_max_dist );
		printf("-- Min dist : %f \n", all_min_dist );
		// cnt = 0;
		int update_min_dist_threshold = 3;
		for (size_t i = 0; i < all_matches.size(); i++) {
			//this->min_dist is just a factor, usually 3
			// threshold ????
					if (all_matches[i].distance <= max(update_min_dist_threshold*min_dist,0.02)) {
							all_good_matches.push_back(all_matches[i]);
							all_PC.push_back(keypointC[all_matches[i].queryIdx]);
							all_last.push_back((last_f->features)[all_matches[i].trainIdx]);
							// all_test_matches.push_back(DMatch(cnt,cnt,1.0)); cnt++;
					}
		}
		std::cout << "threshold suppressed all_matches_C & table size:"<< all_good_matches.size() << std::endl;


		/* IMPORTANT
			all_good_matches.queryIdx -> new view
			all_good_matches.trainIdx -> big table
		*/
		if(DEBUG){
			std::cout << "before....." << std::endl;
				cout<<"featureTable size: "<<(this->featureTable)->rows<<" | "<<(this->featureTable)->cols<<endl;
				cout<<"featureCell size: "<<(this->featureCell)->size()<<endl;
				cout<<"Table Z_i size: "<<(this->Z_i)->size()<<endl;
				cout<<"Table Z_j size: "<<(this->Z_j)->size()<<endl;
				cout<<"Table Z_v size: "<<(this->Z_v)->size()<<endl;
		}
		/*
			UPDATE TABLES!!!
			Z tables
			feature tables
			feature Cells
		*/
		std::vector<unsigned>* Z_i = (this->Z_i);
		std::vector<unsigned>* Z_j = (this->Z_j);
		std::vector<char>* Z_v = (this->Z_v);
		if (!DEBUG) {
			/* check Z tabel */
			for (size_t i = 0; i < Z_i->size(); i++) {
				std::cout << "Z" + to_string(i) +" "<< (*Z_i)[i] <<" "<<(*Z_j)[i]<<" "<<(*Z_v)[i] << std::endl;
			}
		}
		std::vector<unsigned> Z_j_backup = *Z_j;
		// assign Z table that has matches
		std::cout << "featureCell->size(): "<< (this->featureCell)->size() << std::endl;
		unsigned camID = Z_i->back();
		for (size_t i = 0; i < all_good_matches.size(); i++) {
				Z_i->push_back(camID+1);
				Z_j->push_back(all_good_matches[i].trainIdx);
				Z_v->push_back(1);
				//update matched feature desc to nearstest desc
				descC.row(all_good_matches[i].queryIdx).copyTo(((this->featureTable)->row(all_good_matches[i].trainIdx)).colRange(0,128));
				//update 			feature Cells that has matched features
				// Take care of those N to 1 Matches!
				arma::fmat* cell = (*(this->featureCell))[all_good_matches[i].trainIdx];
				int camID_max = std::count (Z_j_backup.begin(), Z_j_backup.end(), all_good_matches[i].trainIdx);
				  // cout<<"size of cell: "<<cell->n_rows<<" | "<< cell->n_cols<<endl;
				if( cell->n_rows < camID_max+1){
					arma::fmat tmp_new_cell_entry(1,2);
					tmp_new_cell_entry(0,0) = all_PC[i].pt.x;
					tmp_new_cell_entry(0,1) = all_PC[i].pt.y;
					cell->insert_rows(cell->n_rows, tmp_new_cell_entry);
					// std::cout << "insert: "<< all_good_matches[i].trainIdx << std::endl;
			}
		}

		if(!DEBUG){
				for (size_t i = 0; i < (this->featureCell)->size(); i++) {
					/* check featureCell */
					std::cout << "featureCell: "<< *(*(this->featureCell))[i] <<"\n"<< std::endl;
				}
		}
		std::cout << "\n\n all_matches size(): "<< all_matches.size() << std::endl;
		// assign Z table that has no match but may match in future frame
		for (size_t i = 0; i < all_matches.size(); i++) {
			/* code */
			if (all_matches[i].distance > max(update_min_dist_threshold*min_dist,0.02)) {
				Z_i->push_back(camID+1);
				Z_j->push_back(Z_j->size());
				Z_v->push_back(1);
				//add correspounding decs match with new Z entry
				Mat new_entry = Mat::zeros(1,(this->featureTable)->cols, CV_32FC1);
				descC.row(all_matches[i].queryIdx).copyTo(new_entry.colRange(0,128));
				(*(this->featureTable)).push_back(new_entry);
				//update feature Cells that has no matched features
				arma::fmat* tmp_feature_per_cell = new arma::fmat(1,2);
				(*tmp_feature_per_cell)(0,0) = keypointC[all_matches[i].queryIdx].pt.x;
				(*tmp_feature_per_cell)(0,1) = keypointC[all_matches[i].queryIdx].pt.y;
				// std::cout << "I add new feature: "<< *tmp_feature_per_cell << std::endl;
				featureCell->push_back(tmp_feature_per_cell);
			}
		}
		if(DEBUG){
				cout<<"featureTable size: "<<(this->featureTable)->rows<<" | "<<(this->featureTable)->cols<<endl;
				cout<<"featureCell size: "<<(this->featureCell)->size()<<endl;
				cout<<"Table Z_i size: "<<(this->Z_i)->size()<<endl;
				cout<<"Table Z_j size: "<<(this->Z_j)->size()<<endl;
				cout<<"Table Z_v size: "<<(this->Z_v)->size()<<endl;
				cout<<"Table camPose size: "<<(this->cameraPose)->size()<<endl;
				cout<<"Table camProjTable size: "<<(this->camProjTable)->size()<<endl;
		}
		/*
			UPDATE TABLES!!!
			done
		*/
		std::cout << "update Tables done!\n" << std::endl;
		arma::umat index_C(all_good_matches.size(),1);
		for (size_t i = 0; i < all_good_matches.size(); i++) {
					index_C(i,0) = all_good_matches[i].trainIdx;
		}
		/*
			triangulation
		*/
		std::cout << "triangulation begin!" << std::endl;
		 multiViewTriangulation(index_C , imC);
		 std::cout << "triangulation end!" << std::endl;

		 arma::fmat featureTable_arma(reinterpret_cast<float*>((*(this->featureTable)).data), 128+6, (*(this->featureTable)).rows);
		 static int debug_cnt = 0;
		 featureTable_arma.save("featureTable_"+to_string(++debug_cnt)+".mat",arma::raw_ascii);
		 std::cout << "index_C, which is used to update size()"<< index_C.size() << std::endl;
		 /*
		 		Debug pose extimations
		 */



			(*(this->camProjTable)).back()->save("camProjTable"+ to_string((*(this->camProjTable)).size()-1) +".mat",arma::raw_ascii);

		 std::cout << "saved camProjTable" << std::endl;

		 /*
		 	Handle last frame
			*/
			last_frame* next_f = new last_frame;
			for (size_t i = 0; i < all_good_matches.size(); i++) {
				/* handle desc */
				(next_f->decs).push_back( (*(this->featureTable)).row(all_good_matches[i].trainIdx).colRange(0,128)  );
				/* handle 3d pts */
				(next_f->pts3D).push_back(  (*(this->featureTable)).row(all_good_matches[i].trainIdx).colRange(128,131));
			}

			/* handle features */
			(next_f->features) = all_PC;

			if(!DEBUG){
				/* DEBUG last frame for future */
				std::cout << "last_frame->all_PC size: "<< all_PC.size() << std::endl;
				//  std::cout << "all_PC: "<< all_PC << std::endl;
				std::cout << "last_frame->features size: "<< (next_f->features).size() << std::endl;
				// std::cout << "feature desc" << (next_f->decs) << std::endl;
			}
			cout<<"last_frame-> features 2D\n";
			// cout<< (next_f->pts3D)<<endl;
			for (size_t i = 0; i < (next_f->features).size(); i++) {
				/* code */
				// cout<< (next_f->features)[i].pt <<endl;
			}
			delete last_f;
			return next_f;
	}
