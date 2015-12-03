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
	vector<arma::fmat*>* camProjTable = this -> camProjTable;
	vector<arma::fmat*>* featureCell = this -> featureCell;
	vector<arma::fmat*>* cameraPose = this -> cameraPose;
	vector<unsigned>* Z_i = this -> Z_i;
	vector<unsigned>* Z_j = this -> Z_j;

	/*cout << "index: " << index << endl;
	cout << "Zi ";
	for(int i = 0; i < (*(this -> Z_i)).size(); i++)
		cout << (*(this -> Z_i))[i] << ' ';
	cout << "\nZj ";
	for(int i = 0; i < (*(this -> Z_j)).size(); i++)
		cout << (*(this -> Z_j))[i] << ' ';*/

	int index_num = index.n_rows;
	for(int i = 0; i < index_num; i++){
		int idx = index(i, 0);
		// 2D point in camera idx
		arma::fmat pts = *(*featureCell)[idx];
		//std::cout << "featureCell" << pts << std::endl;

		// get all the (Z_i, Z_j, Z_v) from camera idx
		vector<int> Z_index;
		for(int j = 0; j < (*Z_j).size(); j++)
			if((*Z_j)[j] == idx && (j == 0 || (*Z_j)[j - 1] != (*Z_j)[j]))
				Z_index.push_back((*Z_i)[j]);

		// get the matrix for SVD decomposition
		int n = Z_index.size();
		arma::fmat A(2 * n, 4);
		for(int j = 0; j < Z_index.size(); j++){
			std::cout << "Z_index" << Z_index[j] << std::endl;
			std::cout << "Proj" << (*(*camProjTable)[Z_index[j]]) << std::endl;
			//A(j, 0) = (*(*camProjTable)[idx])//(2, 0) * pts(0, 0);
		}
	}


	return 1;
};
