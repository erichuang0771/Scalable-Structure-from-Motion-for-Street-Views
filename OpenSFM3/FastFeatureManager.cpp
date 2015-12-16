#include "FastFeatureManager.h"

using namespace std;
using namespace cv;

FastFeatureManager::FastFeatureManager(vector<View>& viewSeq):_viewSeq(viewSeq)
{
	_seqLength = _viewSeq.size();
	_detector = FeatureDetector::create("PyramidFAST");
	_extractor = DescriptorExtractor::create("ORB");

	ExtractAllPoints();

}


void FastFeatureManager::ExtractAllPoints()
{
	cout << " -------------------- extract feature points for all images -------------------\n";
	for( vector<View>::iterator it = _viewSeq.begin();it!=_viewSeq.end();it++)
	{
		//_kptsSeq.push_back(vector<KeyPoint>());
		vector<KeyPoint> kpts;
		Mat descriptor;
		ExtractOneViewPoints(*it,kpts,descriptor);
		_kptsSeq.push_back(kpts);
		_descriptors.push_back(descriptor);
		KeyPointVecToPoint2fVec(kpts,it->_Points2d);
		it->_isTriangulated.assign(it->_Points2d.size(),false);
	}
	cout << " ------------------------------------- done -----------------------------------\n";
}

void FastFeatureManager::ExtractOneViewPoints(View view, vector<KeyPoint>& kpts, Mat& descriptor)
{
	_detector->detect(view._image, kpts);
	_extractor->compute(view._image, kpts, descriptor);

	cout<<"image"<<view._index<<":   "<<kpts.size()<<" keypoints detected"<<endl;

#ifdef _FEATUREDEBUG
	if(view._index==0)
	{
		Mat outputImage;
		drawKeypoints(view._image,kpts,outputImage);
		imshow("FAST KeyPoints",outputImage);
		waitKey(20);
	}

#endif
}


void FastFeatureManager::MatchTwoViews(int idx1, int idx2, vector<DMatch>& outDMatch)
{
	vector<KeyPoint> kpts1 = _kptsSeq[idx1];
	vector<KeyPoint> kpts2 = _kptsSeq[idx2];
    const Mat& descriptors1 = _descriptors[idx1];
    const Mat& descriptors2 = _descriptors[idx2]; 

#ifdef _FEATUREDEBUG   
	cout<< "View"<< idx1<<" has " << kpts1.size() << " points (descriptors " << descriptors1.rows << ")" << endl;
	cout<< "View"<< idx2<<" has " << kpts2.size() << " points (descriptors " << descriptors2.rows << ")" << endl;
#endif

    if(descriptors1.empty()) 
	{
        CV_Error(0,"descriptors1 is empty");
    }
    if(descriptors2.empty()) 
	{
        CV_Error(0,"descriptors2 is empty");
    }
    
    //matching descriptor vectors using Brute Force matcher
    BFMatcher matcher(NORM_HAMMING,true); //allow cross-check. use Hamming distance for binary descriptor (ORB)
	outDMatch.clear();
    matcher.match( descriptors1, descriptors2, outDMatch);    
	assert(outDMatch.size()>0);

   
    
#ifdef _FEATUREDEBUG
    
        Mat matchImage;
        drawMatches( _viewSeq[idx1]._image, kpts1, _viewSeq[idx2]._image, kpts2,
                    outDMatch, matchImage, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );        
		stringstream ss; ss << "Feature Matches " << idx1 << "-" << idx2;
        imshow(ss.str() , matchImage);
        waitKey(20);
#endif

}

void FastFeatureManager::MatchTwoViews(int idx1, int idx2, vector<IdxPair>& outIdxPair)
{
	cout<<"--------------matching image "<<idx1<<" and "<<idx2<<"-------------------------"<<endl;
	vector<DMatch> matches;
	MatchTwoViews(idx1,idx2,matches);
	DMatchVecToIdxPairVec(matches,outIdxPair);
	Mat F = ComputeFAndInliners(idx1,idx2,outIdxPair);
	_FMatMap[make_pair(idx1,idx2)] = F.clone();
	_matchMap[make_pair(idx1,idx2)] = outIdxPair;

	//cout<<outIdxPair.size()<<endl;
	cout<<"----match image "<<idx1<<" and "<<idx2<<" done, "<<outIdxPair.size()<<"Pairs----"<<endl;
}


Mat FastFeatureManager::ComputeFAndInliners(int idx1, int idx2, vector<IdxPair>& idxPair)
{
	vector<int> idxSeq1,idxSeq2;
	SplitIdxPairVec(idxPair,idxSeq1,idxSeq2);

	vector<Point2f> x1,x2;
	_viewSeq[idx1].GetCroodsFromIndexes(idxSeq1,x1);
	_viewSeq[idx2].GetCroodsFromIndexes(idxSeq2,x2);

	vector<uchar> status(idxPair.size());

	Mat F;

	double minVal, maxVal;
	minMaxIdx(x1,&minVal,&maxVal);
	F = findFundamentalMat(x1, x2, FM_RANSAC, 0.006 * maxVal, 0.99, status); //threshold from [Snavely07 4.1]
	//cout<<"F threshold="<<0.006*maxVal<<endl;

	cout << "F keeping " << countNonZero(status) << " / " << status.size() << endl;

	vector<IdxPair> idxPairTemp;
	idxPairTemp = idxPair;
	idxPair.clear();
	for(unsigned int i=0;i<status.size();i++)
	{
		if(status[i])
		{
			idxPair.push_back(idxPairTemp[i]);
		}
	}
	return F;
}

void FastFeatureManager::GetCorrPoints(int idx1, int idx2, vector<IdxPair>& outIdxPair)
{
	if(idx1>idx2)
	{
		GetCorrPoints(idx2,idx1,outIdxPair);
		flipIdxPair(outIdxPair);
		return;
	}
	//see if matching result is stored in matchMap
	map<pair<int,int>,vector<IdxPair>>::iterator it;
	it = _matchMap.find(make_pair(idx1,idx2));

	//if stored, copy that vector to output
	if (it!=_matchMap.end())
	{
		outIdxPair = it->second;
	}
	//if not, do teh matching process and store
	else
	{
#ifdef _FEATUREDEBUG		
		double t = cv::getTickCount();
#endif
		MatchTwoViews(idx1,idx2,outIdxPair);
#ifdef _FEATUREDEBUG
		t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
		cout << "Done. (" << t <<"s)"<< endl;
#endif
	}
}

Mat FastFeatureManager::GetCorrF(int idx1, int idx2)
{
	Mat F;
	if(idx1>idx2)
	{
		F = GetCorrF(idx2,idx1);
		F = F.t();
		//cout<<"F:"<<F<<endl;
		return F;
	}
	map<pair<int,int>,Mat>::iterator it;
	it = _FMatMap.find(make_pair(idx1,idx2));
	if (it!=_FMatMap.end())
	{
		F = (it->second);
	}
	else
	{
		vector<IdxPair> tempIdxPair;
		MatchTwoViews(idx1,idx2,tempIdxPair);
		F = _FMatMap[make_pair(idx1,idx2)];
	}
	//cout<<"F:"<<F<<endl;
	return F;
}