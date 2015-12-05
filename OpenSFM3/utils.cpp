#include "OpenSfM.h"

using namespace std;

bool OpenSfM::loadParas(std::string dir){
	std::string line;
	std::ifstream paraFile(dir);
	if(!paraFile)
		return 0;
	// read intrinsic matrix
	getline(paraFile, line, ':');
	double elem[9];
	for(int i = 0; i < 9; i++){
		if(i < 8)
			getline(paraFile, line, ' ');
		else
			getline(paraFile, line);
		elem[i] = stof(line);
	}
	this -> intrinsc_K
		<< elem[0] << elem[1] << elem[2] << arma::endr
        	<< elem[3] << elem[4] << elem[5] << arma::endr
        	<< elem[6] << elem[7] << elem[8];
	// read parameter min_dist
	getline(paraFile, line, ':');
	getline(paraFile, line);
	this -> min_dist = std::stoi(line);
	// read images
	getline(paraFile, line, ':');
	getline(paraFile, line, ' ');
	int img_num = std::stoi(line);
	getline(paraFile, line);
	for(int i = 5; i <= img_num; i++){
		std::stringstream ss;
		ss << /*std::setw(2) << std::setfill('0') <<*/ i;
		cv::Mat img = cv::imread(line + ss.str() + ".bmp",  CV_LOAD_IMAGE_COLOR);
		this -> images.push_back(img);
	}
	this -> img_rows = (this -> images[0]).rows;
	this -> img_cols = (this -> images[0]).cols;
	return 1;
};

int OpenSfM::multiViewTriangulation(arma::umat& index , cv::Mat& ims){
	vector<arma::fmat*>* camProjTable = this -> camProjTable;
	vector<arma::fmat*>* featureCell = this -> featureCell;
	vector<unsigned>* Z_i = this -> Z_i;
	vector<unsigned>* Z_j = this -> Z_j;
	// cout << "index: " << index << endl;
	//  cout << "Zi ";
	//  for(int i = 0; i < (*(this -> Z_i)).size(); i++)
	//  	cout << (*(this -> Z_i))[i] << ' ';
	//   cout << "\nZj ";
	//  for(int i = 0; i < (*(this -> Z_j)).size(); i++)
	//  	cout << (*(this -> Z_j))[i] << ' ';

	int index_num = index.n_rows;
	for(int i = 0; i < index_num; i++){
		int idx = index(i, 0);
		// 2D point in camera idx
		arma::fmat pts = *(*featureCell)[idx];
		//  std::cout << "featureCell" << pts << std::endl;

		// get all the (Z_i, Z_j, Z_v) from camera idx
		vector<unsigned> Z_index;
		for(int j = 0; j < (*Z_j).size(); j++){
			//cout << "Zj: " << (*Z_i)[j] << "|" << (*Z_j)[j] << endl;
			if((*Z_j)[j] == idx && (Z_index.empty() || (*Z_i)[j] != Z_index.back()))
				Z_index.push_back((*Z_i)[j]);
			}

		// get the matrix for SVD decomposition
		int n = Z_index.size();
		arma::fmat A(2 * n, 4);
		if(pts.n_rows != Z_index.size()){
			std::cout << "error" << std::endl;
			std::cout << "idx: " << pts.n_rows << "|" << Z_index.size() << "|" << idx << std::endl;
			std::cout << "pts" << pts << std::endl;
			vector<unsigned> Z_index_;
			for(int j = 0; j < (*Z_j).size(); j++)
				if((*Z_j)[j] == idx)
					cout << "Zj: " << (*Z_i)[j] << "|" << (*Z_j)[j] << endl;
		}
		for(int j = 0; j < Z_index.size(); j++){
			// std::cout << "Z_index" << Z_index[j] << std::endl;
			A(j, 0) = (*(*camProjTable)[Z_index[j]])(2, 0) * pts(j, 0) - (*(*camProjTable)[Z_index[j]])(0, 0);
			A(j, 1) = (*(*camProjTable)[Z_index[j]])(2, 1) * pts(j, 0) - (*(*camProjTable)[Z_index[j]])(0, 1);
			A(j, 2) = (*(*camProjTable)[Z_index[j]])(2, 2) * pts(j, 0) - (*(*camProjTable)[Z_index[j]])(0, 2);
			A(j, 3) = (*(*camProjTable)[Z_index[j]])(2, 3) * pts(j, 0) - (*(*camProjTable)[Z_index[j]])(0, 3);
			A(n + j, 0) = (*(*camProjTable)[Z_index[j]])(2, 0) * pts(j, 1) - (*(*camProjTable)[Z_index[j]])(1, 0);
			A(n + j, 1) = (*(*camProjTable)[Z_index[j]])(2, 1) * pts(j, 1) - (*(*camProjTable)[Z_index[j]])(1, 1);
			A(n + j, 2) = (*(*camProjTable)[Z_index[j]])(2, 2) * pts(j, 1) - (*(*camProjTable)[Z_index[j]])(1, 2);
			A(n + j, 3) = (*(*camProjTable)[Z_index[j]])(2, 3) * pts(j, 1) - (*(*camProjTable)[Z_index[j]])(1, 3);
		}
	 //std::cout << "A" << A << std::endl;

		// SVD decomposition
		arma::fmat U;
		arma::fvec s;
		arma::fmat V;
		arma::svd(U, s, V, A);
		//std::cout << "V" << V.col(3) << std::endl;

		// update feature table
		// std::cout << "INDEX: "<< idx << std::endl;
		// std::cout << "featureTable size(): "<< featureTable->size() << std::endl;
		// std::cout << "V"<< V << std::endl;
		(*(this -> featureTable)).at<float>(idx, 128) = V(0, 3) / V(3, 3);
		(*(this -> featureTable)).at<float>(idx, 129) = V(1, 3) / V(3, 3);
		(*(this -> featureTable)).at<float>(idx, 130) = V(2, 3) / V(3, 3);
		//  cout<<"hehehehehe\n";
		if((*(this -> featureTable)).at<float>(idx, 131) == 0){
			int camID = pts.n_rows - 1;
			// std::cout << "camID: "<<camID << std::endl;
		  // std::cout << "check: "<< round(pts(camID, 1)) <<" | "<< round(pts(camID, 0))  << std::endl;

			cv::Vec3b rgb = ims.at<cv::Vec3b>(round(pts(camID, 1)), round(pts(camID, 0)));
			(*(this -> featureTable)).at<float>(idx, 131) = rgb.val[2];
			(*(this -> featureTable)).at<float>(idx, 132) = rgb.val[1];
			(*(this -> featureTable)).at<float>(idx, 133) = rgb.val[0];
		}
	}
	//  cout<<"#####\n";
	return 1;
};
