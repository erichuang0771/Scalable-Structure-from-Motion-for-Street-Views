////////////////////////////
//
// the class for feature package, output corr points for two views
//
////////////////////////////

#pragma once


#include <opencv2/opencv.hpp>
#include "Common.h"
#include "View.h"

#include "FeatureManager.h"

using namespace std;
using namespace cv;


class KLTFeatureManager : public FeatureManager
{
public:
	KLTFeatureManager(vector<View>& viewSeq);


	//the only two useful function for other classes
	void GetCorrPoints(int idx1, int idx2, vector<IdxPair>& outIdxPair);
	//void GetCorrPoints(int idx1, int idx2, vector<Point2d>& x1, vector<Point2d>& x2);
	Mat GetCorrF(int idx1, int idx2);

private:

	void TrackOneStep(int idx);
	// FMatMap and matchMap are updated in matchTwoViews
	void MatchTwoViews(int idx1, int idx2, vector<IdxPair>& outIdxPair);

	void MergeIdxPair(vector<IdxPair>& outIdxPair1,vector<IdxPair>& inIdxPair2);

	Mat ComputeFAndInliners(int idx1, int idx2, vector<IdxPair>& idxPair);

	//extract all corner points
	void ExtractAllPoints();

	//track corner points into sequences
	void TrackAllFrames();

	//move main point from image center back to OpenCV coordinate
	void UnMoveMainPoint(vector<Point2f>& points);
	void MoveMainPoint(vector<Point2f>& points);

	//void ExtractOneViewPoints(View view, vector<KeyPoint>& kpts, Mat& descriptor);


	//storing matched feature index pairs
	map<pair<int,int>,vector<IdxPair>> _matchMap;
	//continous tracking sequence
	map<pair<int,int>,vector<IdxPair>> _trackMap;
	//storing F matrix by a pair of matches
	map<pair<int,int>,Mat> _FMatMap;


	//reference of image sequence 
	vector<View>& _viewSeq;
	int _seqLength;

	//Ptr<FeatureDetector> _detector;
	//Ptr<DescriptorExtractor> _extractor;
	//vector<Mat> _descriptors;
	//vector<vector<KeyPoint>> _kptsSeq;
};