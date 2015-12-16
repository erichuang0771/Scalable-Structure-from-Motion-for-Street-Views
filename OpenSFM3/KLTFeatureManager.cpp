#include "KLTFeatureManager.h"


KLTFeatureManager::KLTFeatureManager(vector<View>& viewSeq):_viewSeq(viewSeq)
{
	_seqLength = _viewSeq.size();
	ExtractAllPoints();
	TrackAllFrames();
}

void KLTFeatureManager::ExtractAllPoints()
{
	cout << " -------------------- extract feature points for all images -------------------\n";
	FastFeatureDetector ffd;
	for( vector<View>::iterator it = _viewSeq.begin();it!=_viewSeq.end();it++)
	{
		vector<KeyPoint> kpts;
		ffd.detect(it->_image,kpts);
#ifdef _FEATUREDEBUG
		if(it->_index==0)
		{
			Mat outputImage;
			drawKeypoints(it->_image,kpts,outputImage);
			imshow("KLT KeyPoints",outputImage);
			waitKey(20);
		}

#endif
		//_kptsSeq.push_back(kpts);
		KeyPointVecToPoint2fVec(kpts,it->_Points2d);
		it->_isTriangulated.assign(it->_Points2d.size(),false);
	}

	cout << " ------------------------------------- done -----------------------------------\n";
}

void KLTFeatureManager::UnMoveMainPoint(vector<Point2f>& points)
{
	for(int i=0;i<points.size();i++)
	{
		points[i].x = points[i].x + 720/2.0;
		points[i].y = points[i].y+576/2.0;
	}
}
void KLTFeatureManager::MoveMainPoint(vector<Point2f>& points)
{
	for(int i=0;i<points.size();i++)
	{
		points[i].x = points[i].x - 720/2.0;
		points[i].y = points[i].y-576/2.0;
	}
}

void KLTFeatureManager::TrackOneStep(int idx)
{
	//no Track for the last frame
	if(idx >= _viewSeq.size()-1)
	{
		return;
	}

	//change RGB frame into gray frame
	Mat image1,image2;
	if(_viewSeq[idx]._image.channels()==3)
	{
		cvtColor(_viewSeq[idx]._image,image1,CV_RGB2GRAY);
		cvtColor(_viewSeq[idx+1]._image,image2,CV_RGB2GRAY);
	}
	else
	{
		image1 = _viewSeq[idx]._image;
		image2 = _viewSeq[idx+1]._image;
	}
	//extract 2d feature points
	vector<Point2f> x1 = _viewSeq[idx]._Points2d;
	UnMoveMainPoint(x1);
	vector<Point2f> xNew(x1.size());
	vector<Point2f> x2 = _viewSeq[idx+1]._Points2d; 
	UnMoveMainPoint(x2);
#ifdef _FEATUREDEBUG   
	cout<< "View"<< idx  <<" has " << x1.size() << " points  "<< endl;
	cout<< "View"<< idx+1<<" has " << x2.size() << " points  "<< endl;
#endif
	//KLT tracking
	vector<uchar> vstatus(x1.size()); vector<float> verror(x1.size());
	calcOpticalFlowPyrLK(image1, image2, x1, xNew, vstatus, verror);

	MoveMainPoint(xNew);
	_viewSeq[idx+1]._Points2d = xNew;

	////extract successful tracks
	//double threshold = 12.0;
	//vector<Point2f> to_find;
	//vector<int> IdxForx1;
	//vector<Point2f> xNewClone = xNew;
	//xNew.clear();
	//for (unsigned int i=0; i<vstatus.size(); i++) 
	//{
	//	if (vstatus[i] && verror[i] < threshold) 
	//	{
	//		IdxForx1.push_back(i);
	//		xNew.push_back(xNewClone[i]);
	//	} 
	//	else 
	//	{
	//		vstatus[i] = 0;
	//	}
	//}
	//cout<<"one step tracks on frame "<<idx<<" : "<<xNew.size()<<"points succeed"<<endl;

	////match KLT tracking result with features on the second image
	//Mat xNewMat = Mat(xNew).reshape(1,xNew.size());
	//Mat x2Mat = Mat(x2).reshape(1,x2.size());
	//vector<vector<DMatch> > knn_matches;
	////FlannBasedMatcher matcher;
	//BFMatcher matcher(CV_L2);
	//matcher.radiusMatch(xNewMat,x2Mat,knn_matches,12.0f);

	//std::set<int> justForPreventDuplicates;
	//vector<DMatch> matches;
	//for(int i=0;i<knn_matches.size();i++) 
	//{
	//	DMatch _m;
	//	if(knn_matches[i].size()==1) 
	//	{
	//		_m = knn_matches[i][0];
	//	} 
	//	else if(knn_matches[i].size()>1) 
	//	{
	//		if(knn_matches[i][0].distance / knn_matches[i][1].distance < 0.7) 
	//		{
	//			_m = knn_matches[i][0];
	//		} 
	//		else 
	//		{
	//			continue; // did not pass ratio test
	//		}
	//	} 
	//	else 
	//	{
	//		continue; // no match
	//	}
	//	if (justForPreventDuplicates.find(_m.trainIdx) == justForPreventDuplicates.end()) { // prevent duplicates
	//		_m.queryIdx = IdxForx1[_m.queryIdx]; //back to original indexing of points for <i_idx>
	//		matches.push_back(_m);
	//		justForPreventDuplicates.insert(_m.trainIdx);
	//	}
	//}	
	//cout << "pruned " << matches.size() << " / " << knn_matches.size() << " matches" << endl;

	//vector<IdxPair> featureIdxPair;
	//DMatchVecToIdxPairVec(matches,featureIdxPair);

	//store tracking result
	//_trackMap[make_pair(idx,idx+1)] = featureIdxPair;

}

void KLTFeatureManager::TrackAllFrames()
{
	for(int i=0;i<_viewSeq.size()-1;i++)
	{
		TrackOneStep(i);
	}

}

void KLTFeatureManager::MergeIdxPair(vector<IdxPair>& outIdxPair1,vector<IdxPair>& inIdxPair2)
{
	vector<IdxPair> result;
	for(vector<IdxPair>::iterator it1=outIdxPair1.begin();it1!=outIdxPair1.end();it1++)
	{
		int target = it1->idx2;
		for(vector<IdxPair>::iterator it2=inIdxPair2.begin();it2!=inIdxPair2.end();it2++)
		{
			if(it2->idx1==target)
			{
				IdxPair temp={it1->idx1,it2->idx2};
				result.push_back(temp);
			}
		}
	}
	outIdxPair1.clear();
	outIdxPair1=result;
}

void KLTFeatureManager::MatchTwoViews(int idx1, int idx2, vector<IdxPair>& outIdxPair)
{
	cout<<"--------------matching image "<<idx1<<" and "<<idx2<<"-------------------------"<<endl;
	//outIdxPair = _trackMap[make_pair(idx1,idx1+1)];
	//int midIdx = idx1+1;
	//while(midIdx<idx2)
	//{
	//	midIdx++;
	//	MergeIdxPair(outIdxPair,_trackMap[make_pair(midIdx,midIdx+1)]);
	//}
	outIdxPair.clear();
	for(int i=0;i<_viewSeq[idx1]._Points2d.size();i++)
	{
		IdxPair temp = {i,i};
		outIdxPair.push_back(temp);
	}
	_trackMap[make_pair(idx1,idx2)]=outIdxPair;
	Mat F = ComputeFAndInliners(idx1,idx2,outIdxPair);
	_FMatMap[make_pair(idx1,idx2)] = F.clone();
	_matchMap[make_pair(idx1,idx2)] = outIdxPair;

	//cout<<outIdxPair.size()<<endl;
	cout<<"----match image "<<idx1<<" and "<<idx2<<" done, "<<outIdxPair.size()<<"Pairs----"<<endl;
}

Mat KLTFeatureManager::ComputeFAndInliners(int idx1, int idx2, vector<IdxPair>& idxPair)
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

void KLTFeatureManager::GetCorrPoints(int idx1, int idx2, vector<IdxPair>& outIdxPair)
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
	//if not, do the matching process and store
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

Mat KLTFeatureManager::GetCorrF(int idx1, int idx2)
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