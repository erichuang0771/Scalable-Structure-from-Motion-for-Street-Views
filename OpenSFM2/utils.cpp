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
	for(int i = 1; i <= img_num; i++){
		std::stringstream ss;
		ss << std::setw(2) << std::setfill('0') << i;
		cv::Mat img = cv::imread(line + ss.str() + ".png",  CV_LOAD_IMAGE_COLOR);
		this -> images.push_back(img);
	}
	
	return 1;
};

int OpenSfM::multiViewTriangulation(arma::umat& index , cv::Mat& ims){
	cout << "index: " << index << endl;
	cout << "Zi" << this -> Z_i << endl;
	cout << "Zj" << this -> Z_j << endl;
	return 1;
};
