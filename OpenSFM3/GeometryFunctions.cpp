#include "GeometryFunctions.h"

using namespace cv;
using namespace std;

bool AllPossiblePFromF(Mat& F, Mat& K, vector<arma::fmat>& Proj_camB){

	cout<<"connect to GeometryFunctions.cpp"<<endl;
	Proj_camB.clear();
	cout<<F.rows<<" | "<<F.cols<<" | "<<F.depth()<<endl;
	cout<<K.rows<<" | "<<K.cols<<" | "<<K.depth()<<endl;
	Mat E = K.t()*F*K;

	cout<<"org: E: "<<E<<endl;
	cout<<"org: F: "<<F<<endl;

	if(fabsf(determinant(E)) > 1e-02){
		cout<<"det(E) != 0 : "<<determinant(E)<<endl;
		return false;
	}

	arma::fmat R1, R2, t1, t2;
	if(!DecomposeEtoRandT(E,R1,R2,t1,t2)){
		cout<< "cannot decompose E!" <<endl;
	}
	if (DEBUG)
	{
		/* code */
		cout<<E<<endl
		<<R1<<endl
		<<R2<<endl
		<<t1<<endl
		<<t2<<endl;
	}
	if(!checkRotationMatrix(R1)){
		cout<<"get a wrong invalid matrix!"<<endl;
		return false;
	}
	arma::fmat Proj_camB_1,Proj_camB_2,Proj_camB_3,Proj_camB_4;
	Proj_camB_1 = arma::join_rows(R1,t1);
	Proj_camB_2 = arma::join_rows(R2,t1);
	Proj_camB_3 = arma::join_rows(R1,t2);
	Proj_camB_4 = arma::join_rows(R2,t2);
	Proj_camB.push_back(Proj_camB_1);
	Proj_camB.push_back(Proj_camB_2);
	Proj_camB.push_back(Proj_camB_3);
	Proj_camB.push_back(Proj_camB_4);
	return true;
}

bool DecomposeEtoRandT(cv::Mat& E, arma::fmat& R1, arma::fmat& R2, arma::fmat& t1, arma::fmat& t2){
	// svd using openCV
	SVD svd(E,SVD::MODIFY_A);

	//check if first and second singleuar value are the same (as they should be)
	double singular_values_ratio = fabsf(svd.w.at<double>(0) / svd.w.at<double>(1) );
	std::cout << "svd:: singleuar "<< svd.w.at<double>(0) <<"  |  " <<svd.w.at<double>(1) << std::endl;
	if(singular_values_ratio > 1.0) singular_values_ratio = 1.0/singular_values_ratio;// filp ration keep it [0,1]
	if(singular_values_ratio < 0.7) {
		cout<<"singular values are too far away"<<endl;
		return false;
	}

	double m = (svd.w.at<double>(0,0)+svd.w.at<double>(1,1))/2;
	Mat s = Mat::zeros(3,3,CV_64FC1);
	s.at<double>(0,0) = m;
	s.at<double>(1,1) = m;
	E = svd.u*s*svd.vt;
	svd(E,SVD::MODIFY_A);

	arma::mat W(3,3);
	W << 0.0 << -1.0 << 0.0 << arma::endr
	  << 1.0 << 0.0 << 0.0 << arma::endr
	  << 0.0 << 0.0 << 1.0 ;
	arma::mat Wt(3,3);
	Wt << 0.0 << 1.0 << 0.0 << arma::endr
	  << -1.0 << 0.0 << 0.0 << arma::endr
	  << 0.0 << 0.0 << 1.0 ;

	 arma::mat svd_u( reinterpret_cast<double*>(svd.u.data), svd.u.rows, svd.u.cols );
	 svd_u = svd_u.t();
	 arma::mat svd_vt( reinterpret_cast<double*>(svd.vt.data), svd.vt.rows, svd.vt.cols );
	 svd_vt = svd_vt.t();
	 cout<<"fuck!!!!!\n";
	 //cout<<K<<endl;
	 //cout<<svd_vt<< endl <<svd.vt<<endl;
	 //
	 if(det(svd_u*W*svd_vt) < 0){
	 	W = -W;
	 	Wt = -Wt;
	 }

	arma::mat R1d = svd_u*W*svd_vt;
	R1 = arma::conv_to<arma::fmat>::from(R1d);
	arma::mat R2d = svd_u*Wt*svd_vt;
	R2 = arma::conv_to<arma::fmat>::from(R2d);

	arma::mat t1d = arma::mat(svd_u.col(2));
	t1 = arma::conv_to<arma::fmat>::from(t1d);

	arma::mat t2d = -arma::mat(svd_u.col(2));
	t2 = arma::conv_to<arma::fmat>::from(t2d);
	return true;
}


bool checkRotationMatrix(arma::fmat& R){
	if(fabsf(det(R) - 1.0) > 1e-04){
		cerr<<"det(R) != +- 1.0, not valid rotation matrix: "<<det(R)<<endl;
		return false;
	}
	return true;

}
