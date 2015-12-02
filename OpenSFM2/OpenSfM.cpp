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
	last_frame* next_f = updateStruture(images[2],last_f);
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
	  Mat fundamental_matrix_ = findFundamentalMat(P1f, P2f, FM_LMEDS, 0.1, 0.99, mask);
		arma::fmat hehehe; hehehe.load("FFF.mat",arma::raw_ascii);
		Mat fundamental_matrix( 3, 3, CV_32FC1, hehehe.memptr() );
		fundamental_matrix.convertTo(fundamental_matrix,CV_64FC1);
		fundamental_matrix = fundamental_matrix.t();
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

	  	Mat Epilines_show = imB;
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
	  this->featureCell = new vector<arma::umat*>();
	  this->cameraPose = new vector<arma::fmat*>();
	  this->camProjTable = new vector<arma::fmat*>();
	  this->Z_i = new vector<unsigned>(num_of_feature);
	  iota((this->Z_i)->begin(),(this->Z_i)->end(),0);
	  this->Z_j = new vector<unsigned>(num_of_feature);
	  iota((this->Z_j)->begin(),(this->Z_j)->end(),0);
	  this->Z_v = new vector<unsigned>(num_of_feature,1);


	  if(DEBUG){
			cout<<"depth of mask: "<<mask.depth()<<endl;
			cout<<"# of inliners: "<<NUM<<endl;
			cout<<"# of all_features: "<<num_of_feature<<endl;
		}

	  for(unsigned i = 0; i < num_of_feature; ++i) {
			/* insert tabel */
	  		//1.copy desc of camB to the table for future match
	  		descB.row(good_matches[i].trainIdx).copyTo(((this->featureTable)->row(i)).colRange(0,128));
	  		//if(DEBUG) cout<<(this->featureTable)->row(i).colRange(0,2)<<" | "<<descB.row(good_matches[i].trainIdx).colRange(0,2)<<endl;

	  		//2.insert correspounding 2D points to feature cell
	  		arma::umat* tmp_feature_per_cell = new arma::umat(2,2);
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

				/*
					prepare BA
				*/

	 arma::fmat point2DA( P1_inliers.size() , 2 );
	 arma::fmat point2DB( P1_inliers.size() , 2 );

	 for(unsigned i = 0; i < P1_inliers.size(); ++i) {
	 	/* code */
	 	point2DA.at(i,0) = P1_inliers[i].x;
	 	point2DA.at(i,1) = P1_inliers[i].y;
	 	point2DB.at(i,0) = P2_inliers[i].x;
	 	point2DB.at(i,1) = P2_inliers[i].y;
	}

	 arma::fmat point4D_( reinterpret_cast<float*>(point4D[best_index].data), point4D[best_index].cols, point4D[best_index].rows );
	 point4D_ = point4D_.t();
	 point4D_.save("final_pts.mat",arma::raw_ascii);
	 cout<<"ready BA: "<<point2DA.n_rows<<point2DB.n_rows<<point4D_.n_rows<<endl;
	//  cout<<"pose: "<<*(*cameraPose)[0]<<"\n pose: "<<*(*cameraPose)[1]<<endl;


	//  local_bundle_adjustment( intrinsc_K,
  //                           *(*cameraPose)[0],
  //                           *(*cameraPose)[1],
  //                           point2DA,
  //                           point2DB,
  //                           point4D_);
	/*
			handle last frame
	*/

	last_frame* last_f = new last_frame;
	last_f->features = point2DB;
	last_f->decs = Mat(P1_inliers.size(),128,descB_candidate.depth());
	cout<<"handle length!\n";
	int ccnt = 0;
	for (size_t i = 0; i < mask.rows; i++) {
		/* copy final desc */
			if(mask.at<char>(i,0) != 0){
			 	descB_candidate.row(i).copyTo((last_f->decs).row(ccnt)); ccnt++;
			}
	}
	last_f->length = arma::fmat(P1_inliers.size(),1);
	for (size_t i = 0; i < P1_inliers.size(); i++) {
		/* compute length */
		(last_f->length).row(i) = sqrt(point4D_(i,0)*point4D_(i,0) + point4D_(i,1)*point4D_(i,1) + point4D_(i,2)*point4D_(i,2));
	}
	cout<<"initalTwoViewRecon done\n";
	if(!DEBUG){
		// cout<<"last_f->features: \n"<<last_f->features<<endl;
		// cout<<"last_f->decs: \n"<<last_f->decs<<endl;
		// cout<<"last_f->length: \n"<<last_f->length<<endl;
	}
	return last_f;
	}

last_frame* OpenSfM::updateStruture(cv::Mat& ims, last_frame* last_frame ){
		std::cout << "connected updateStruture!..." << std::endl;
		
	}
