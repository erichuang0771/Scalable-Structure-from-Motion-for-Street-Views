#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/stitching/stitcher.hpp"
#include <cstdio>
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "pnmfile.h"
#include "segment-image.h"
#include "uFind.h"
#include "create_mid_points.h"
#include "stitching_two.h"
#include "cutting.h"

/*	To do:
 * 1) Clean this code! ---->? how about learning Github?
 * 2) Image blending method? ---->done 10.01.2015
 * 3) New segmentation method?
 * 4) pgm --> ppm --> jpg?
 * 5) download test dateset!
 * 6) Why I cannot prefectly stitch the bg -----> 15.01.2015
 * 7) Color Image
 *
 */

using namespace cv;
std::vector<DMatch> good_matches;
std::vector<Point2f> obj;
std::vector<Point2f> scene;
std::vector<KeyPoint> keypoints_object, keypoints_scene, keypoints_object_org,
    keypoints_scene_org;
Mat img_object;
Mat img_scene;
Mat img_seg;
int y = 500;
cv::Size sze;
void readme();
std::vector<Point2f> middle;


void show(std::vector<Point2f> middle) {

    cv::Mat result;
    Mat Hl = findHomography(obj, middle, CV_RANSAC);
    Mat Hr = findHomography(scene, middle, CV_RANSAC);

    warpPerspective(img_object, result, Hl,
                    cv::Size(img_object.cols, img_object.rows));
    cv::Mat temp;
    warpPerspective(img_scene, temp, Hr,
                    cv::Size(img_object.cols, img_object.rows));

    int threshold_value = 20;
    int max_BINARY_value = 256;
    Mat mask;
    threshold(temp, mask, threshold_value, max_BINARY_value, THRESH_BINARY);
    temp.copyTo(result, mask);
    imshow("RESULT", result);
}

static void M(int, void*) {
    cv::Mat result;

    std::vector<Point2f> middle;
    for (int i = 0; i < obj.size(); i++) {
        float x = y * 1.0 / 1000;
        middle.push_back((obj[i] * x + scene[i] * (1 - x)));
        //std::cout<<middle[i]<<"\n";
    }

    Mat Hl = findHomography(obj, middle, CV_RANSAC);
    Mat Hr = findHomography(scene, middle, CV_RANSAC);

    warpPerspective(img_object, result, Hl,
                    cv::Size(img_object.cols, img_object.rows));
    cv::Mat temp;
    warpPerspective(img_scene, temp, Hr,
                    cv::Size(img_object.cols, img_object.rows));
    std::cout << y << "\n\n";

    int threshold_value = 20;
    int max_BINARY_value = 256;
    Mat mask;
    threshold(temp, mask, threshold_value, max_BINARY_value, THRESH_BINARY);
    temp.copyTo(result, mask);
    imshow("RESULT", result);

}


/** @function main */

int main(int argc, char** argv) {
    if (argc != 5) {
        readme();
        return -1;
    }
    image<rgb> *img_object_ppm = loadPPM(argv[1]);
    image<rgb> *img_scene_ppm = loadPPM(argv[2]);

    Mat img_object_org = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_scene_org = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    if (!img_object_org.data || !img_scene_org.data) {
        std::cout << " --(!) Error reading images " << std::endl;
        return -1;
    }

    //-- Step 1: Detect the keypoints using SFT Detector
    //int minHessian = 400; //1400 for m1m2 3&4; 400 for 1&2; 450 for m3 m4

    SiftFeatureDetector detector;

    detector.detect(img_object_org, keypoints_object);
    detector.detect(img_scene_org, keypoints_scene);

    //--STEP 1.5   create the boarder
    for (int i = 0; i < keypoints_scene.size(); i++) {
        {
            keypoints_scene[i].pt.x = keypoints_scene[i].pt.x
                                      + img_object_org.cols * (0.5);
            keypoints_scene[i].pt.y = keypoints_scene[i].pt.y
                                      + img_object_org.rows * (0.5);
        }
    }
    for (int i = 0; i < keypoints_object.size(); i++) {
        {
            keypoints_object[i].pt.x = keypoints_object[i].pt.x
                                       + img_object_org.cols * (0.5);
            keypoints_object[i].pt.y = keypoints_object[i].pt.y
                                       + img_object_org.rows * (0.5);
        }
    }

    Size sz;
    img_object = Mat::zeros(img_object_org.rows * 2, img_object_org.cols * 2,
                      CV_8UC3);
    img_scene = Mat::zeros(img_object_org.rows * 2, img_object_org.cols * 2, CV_8UC3);
    
    //handle::
    Mat ROI_obj = img_object(
                      cv::Rect(img_object_org.cols * (0.5), img_object_org.rows * (0.5),
                               img_object_org.cols, img_object_org.rows));
    Mat ROI_scene = img_scene(
                        cv::Rect(img_object_org.cols * (0.5), img_object_org.rows * (0.5),
                                 img_object_org.cols, img_object_org.rows));

    img_object_org.copyTo(ROI_obj);
    img_scene_org.copyTo(ROI_scene);
    ROI_obj.release();
    ROI_scene.release();

    //-- Step 2: Calculate descriptors (feature vectors)
    SiftDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute(img_object, keypoints_object, descriptors_object);
    extractor.compute(img_scene, keypoints_scene, descriptors_scene);

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match(descriptors_object, descriptors_scene, matches);

    double max_dist = 0;
    double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < descriptors_object.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )

    for (int i = 0; i < descriptors_object.rows; i++) {
        if ((matches[i].distance < 3 * min_dist)) {
            good_matches.push_back(matches[i]);
        }
    }
    std::cout<<"initial macthing points: "<<good_matches.size()<<"\n\n###########";
       show_match(img_object, img_scene, keypoints_object,  keypoints_scene, good_matches);
	descriptors_object.release();
	descriptors_scene.release();
/*
 * Create Segments!
 * Double check what these parameters mean?
 */
    int num_ccs;
    float sigma = 0.5;
    int k = 1000;
    int min_size = atoi(argv[4]);
    std::cout<<"\n\nSeg # of Img_A ";
    std::map<int, image<rgb>* > map_A = segment_image(img_object_ppm, sigma, k, min_size, &num_ccs );
    std::cout<<"\n\nSeg # of Img_B ";
    std::map<int, image<rgb>* > map_B = segment_image(img_scene_ppm, sigma, k, min_size, &num_ccs);
    universe *uA = uFind(img_object_ppm, sigma, k, min_size, &num_ccs);
    universe *uB = uFind(img_scene_ppm, sigma, k, min_size, &num_ccs);
    std::vector<seg> seg_A;
    std::vector<seg> seg_B;
    vector<DMatch>::iterator i = good_matches.begin();
    while(i != good_matches.end()) {
        int x = keypoints_object[i->queryIdx].pt.x - img_object_org.cols * (0.5) ;// - img_object_org.cols * (0.5);
        int y = keypoints_object[i->queryIdx].pt.y - img_object_org.rows * (0.5);// - img_object_org.rows * (0.5);
        int ID = uA->find(y*img_object_ppm->width() + x);
        std::map<int, image<rgb>* >::iterator iter = map_A.find(ID);
        if (iter==map_A.end()) {
            std::cout<<"map A error!"<<ID<<"\n";
            i++;
            continue;
        }
        int j = 0;
        int flag = 0;
        while (j < seg_A.size())
        {
            if (seg_A[j].ID == ID)
            {
                seg_A[j].match.push_back(*i);
                good_matches.erase(i);							//-------------------------------------->>>  Erase 1
                flag = 1;
               // std::cout<<"add a new point\n";
                break;
            }
            j++;
        }
        if (flag != 1) {
            seg temp;
            temp.ID = iter->first;
            temp.IMG = iter->second;
            temp.match.push_back(*i);
            good_matches.erase(i);							 //-------------------------------------->>>  Erase 2
            seg_A.push_back(temp);
            //std::cout<<"\nadd a new seg\n";
           // i++;
        }
    }
    std::cout<<"\nIMG_A DONE!"<<seg_A.size()<<"-----"<<map_A.size()<<"\n";
    /*
     * Filter , delete all segs that has less than 4 prefect matches
     */
    std::vector<seg>::iterator it = seg_A.begin();
    int count = 0;
    while(it != seg_A.end())
    {

        std::vector<DMatch>::iterator iter = it->match.begin();
        int max = -1;
        int match_ID = -1;
        std::map<int, int> adder;
        while(iter != it->match.end())
        {
            int x = keypoints_scene[iter->trainIdx].pt.x - img_object_org.cols * (0.5) ;// - img_object_org.cols * (0.5);
            int y = keypoints_scene[iter->trainIdx].pt.y - img_object_org.rows * (0.5);// - img_object_org.rows * (0.5);
            int ID = uB->find(y*img_object_ppm->width() + x);
            std::map<int,int>::iterator i = adder.find(ID);
            if(i==adder.end())
            {
                int a = 0;
                adder.insert(std::pair<int,int>(ID, a));
            }
            else
            {
                i->second++;
                if (i->second > max)
                {
                    max = i->second;
                    match_ID = i->first;
                }
            }
            iter++;
        }
        //std::cout<<"match_ID="<<match_ID<<"\n";
        it->match_ID = match_ID;

        iter = it->match.begin();
        while (iter != it ->match.end())
        {
            int x = keypoints_scene[iter->trainIdx].pt.x - img_object_org.cols * (0.5) ;// - img_object_org.cols * (0.5);
            int y = keypoints_scene[iter->trainIdx].pt.y - img_object_org.rows * (0.5);// - img_object_org.rows * (0.5);
            int ID = uB->find(y*img_object_ppm->width() + x);
            if(ID != it->match_ID)
            {
                it->match.erase(iter);
                // std::cout<<"delete a point!\n";
            }
            else iter++;
        }
      
        if(it->match.size()<4)
        {
            int i = 0;
            while(i<it->match.size()) {
                good_matches.push_back(it->match[i]);							//-------------------------------------->>>  PushBack 1
                i++;
            }
            std::cout<<"delete one seg!"<<it->match.size()<<"\n";
            seg_A.erase(it);
            continue;
        }
		count++;
        it++;
    }
    std::cout<<"final:"<<seg_A.size()<<"\n";

    Mat img_matches;
    //////////////////////////////////////////////////////
    std::vector<seg >::iterator itx = seg_A.begin();
    while( itx!=seg_A.end())
    {
        Mat img_matches;
        drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                    itx->match, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        pyrDown(img_matches, img_matches, sz, BORDER_DEFAULT);
        pyrDown(img_matches, img_matches, sz, BORDER_DEFAULT);
        imshow("Good Matches & Object detection----> use \"d\" to delete matches!", img_matches);
        char key = cvWaitKey();
        if(key == 'd') {
            int i = 0;
            while(i<itx->match.size()) {
                good_matches.push_back(itx->match[i]);							//-------------------------------------->>>  PushBack 2
                i++;
            }
            seg_A.erase(itx);

            std::cout<<"manually delete one!\n";
        }
        else itx++;
    }

    itx = seg_A.begin();
    while( itx!=seg_A.end())
    {
        std::map<int, image<rgb>* >::iterator iter = map_B.find(itx->match_ID);
        std::cout<<itx->match_ID<<"\n";
        if (iter==map_B.end()) {
            std::cout<<"map B error!\n";
            itx++;
            continue;
        }
        seg temp;
        temp.IMG = iter->second;
        seg_B.push_back(temp);
        itx++;
    }
    std::cout<<"final's final # of seg_A = "<<seg_A.size()<<"\n";
    std::cout<<"final's final # of seg_B = "<<seg_B.size()<<"\n";
    /*Next step:
     *
     * seg_A ppm --> Mat
     * seg_B ppm --> Mat
     *
     * making mask
     *
     * stitching background
     *
     * stitching object
     */
    int counter = 0;
    for( std::vector<seg >::iterator it = seg_A.begin(); it!=seg_A.end(); it++)
    {
        std::cout<<"did one A!\n";
        char s[10];
        std::sprintf(s,"%d.ppm",counter++);
        savePPM(it->IMG,s);
        Mat temp =  imread(s, CV_LOAD_IMAGE_COLOR);
        std::remove(s);
        Mat final;
        final = Mat::zeros(img_object_org.rows * 2, img_object_org.cols * 2, CV_8UC3);
        Mat ROI_final = final(
                            cv::Rect(img_object_org.cols * (0.5), img_object_org.rows * (0.5),
                                     img_object_org.cols, img_object_org.rows));
        temp.copyTo(ROI_final);
        it->img = final;
    }
    counter = 0;
    for( std::vector<seg >::iterator it = seg_B.begin(); it!=seg_B.end(); it++)
    {
        std::cout<<"did one B!\n";
        char s[10];
        std::sprintf(s,"%d.ppm",counter++);
        savePPM(it->IMG,s);
        Mat temp =  imread(s, CV_LOAD_IMAGE_COLOR);
        std::remove(s);
        Mat final;
        final =  Mat::zeros(img_object_org.rows * 2, img_object_org.cols * 2, CV_8UC3);
        Mat ROI_final = final(
                            cv::Rect(img_object_org.cols * (0.5), img_object_org.rows * (0.5),
                                     img_object_org.cols, img_object_org.rows));
        temp.copyTo(ROI_final);
        it->img = final;
    }
    /*
     * check! can delete!
     */
    std::cout<<"checking time!";
    itx = seg_A.begin();
    int index = 0;
    while( itx!=seg_A.end())
    {
        Mat img_matches;
        drawMatches(itx->img, keypoints_object, seg_B[index].img, keypoints_scene,
                    itx->match, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        index = index + 1;
        pyrDown(img_matches, img_matches, sz, BORDER_DEFAULT);
        pyrDown(img_matches, img_matches, sz, BORDER_DEFAULT);
        imshow("Good Matches & Object detection", img_matches);
        char key = cvWaitKey(300);
        itx++;
    }
    /*
     * start stitching --> Background !
     */
    int cnt = 0;
    float ratio = 0;
    int num = atoi(argv[3]);
    while(cnt < num) {
        std::cout<<"stitching background!\n";
        cv::Mat result_A(img_object.rows, img_object.cols, CV_8UC3);
        std::vector<seg >::iterator its = seg_A.begin();
        result_A = img_object;
        while( its!=seg_A.end()) {
            result_A = cutting( result_A,  its->img,  img_object);
            its++;
        }
        cv::Mat result_B(img_object.rows, img_object.cols, CV_8UC3);
        its = seg_B.begin();
        result_B = img_scene;
        while( its!=seg_B.end()) {
            result_B = cutting( result_B,  its->img,  img_object);
            its++;
        }
        /*
         * delete matching points?
         */
        match_seg mid;
        mid = find_mid_point(good_matches, ratio,keypoints_object, keypoints_scene);
        Mat result;

        show_match(result_A, result_B, keypoints_object,  keypoints_scene, good_matches);

        result = stitching_bg( mid, result_A,  result_B, img_object);
        /*
         * start stitching!--> Object!
         */
        int seg_index = 0; Mat show;
        while(seg_index < seg_A.size()) {
            mid = find_mid_point(seg_A[seg_index].match, ratio,keypoints_object, keypoints_scene);
            result = stitching_two( mid, seg_A,  seg_B, img_object, seg_index, result);
            
           pyrDown(result, show, sz, BORDER_DEFAULT);
			pyrDown(show, show, sz, BORDER_DEFAULT);
            imshow("result!!!", show);
            waitKey(300);
            std::cout<<"stitching one! \n";
            seg_index++;
        }

        pyrDown(result, result, sz, BORDER_DEFAULT);
        //pyrDown(result, result, sz, BORDER_DEFAULT);
        char s[10];
        std::sprintf(s,"result: %d.jpg",cnt);
        imwrite(s, result);
        //waitKey(0);
        ratio = ratio + 1.0/num;
        cnt++;
    }
    exit(0);

    ///////key

    static int degree = 0;
    static int pitch = 0;
    std::vector<Point2f> middle_org = middle;
    ///INit:

    int z = 3000;
    int f = 1500;
    while (true) {
        //horizontal
        Mat R = Mat::zeros(3, 3, CV_32F);
        R.at<float>(0, 0) = cos(degree * 1.0 / 180 * 3.14);
        R.at<float>(0, 2) = sin(degree * 1.0 / 180 * 3.14);
        R.at<float>(2, 0) = -sin(degree * 1.0 / 180 * 3.14);
        R.at<float>(2, 2) = cos(degree * 1.0 / 180 * 3.14);
        R.at<float>(1, 1) = 1;
        //vertical
        Mat V = Mat::zeros(3, 3, CV_32F);
        V.at<float>(1, 1) = cos(pitch * 1.0 / 180 * 3.14);
        V.at<float>(2, 1) = sin(pitch * 1.0 / 180 * 3.14);
        V.at<float>(1, 2) = -sin(pitch * 1.0 / 180 * 3.14);
        V.at<float>(2, 2) = cos(pitch * 1.0 / 180 * 3.14);
        V.at<float>(0, 0) = 1;
        //
        Mat P(3, 1, CV_32F);
        char key = cvWaitKey();
        switch (key) { ///////////////////////////////////////find me the equation!!!!
        case 'a': // turn right;
            if (degree < 90)
                degree = degree + 5;

            break;
        case 'd':
            if (degree > -90)
                degree = degree - 5;

            break;
        case 's':
            if (pitch < 90)
                pitch = pitch + 5;

            break;
        case 'w':
            if (pitch > -90)
                pitch = pitch - 5;
            break;
        }
        for (int i = 0; i < obj.size(); i++) {
            P.at<float>(0, 0) = (img_object.cols / 2 - middle_org[i].x) * 1.0 * z / f;
            P.at<float>(1, 0) = (img_object.rows / 2 - middle_org[i].y) * 1.0 * z / f;
            P.at<float>(2, 0) = z;
            Mat x = V * R * P;
            float zc = x.at<float>(2, 0);
            middle[i].x = (-x.at<float>(0, 0) * 1.0 * f / zc
                           + img_object.cols / 2);
            middle[i].y = (-x.at<float>(1, 0) * 1.0 * f / zc
                           + img_object.rows / 2);
        }
        show(middle);
    }
    return 0;

}

/** @function readme */
void readme() {
    std::cout << " Usage: ./ImageD <img1> <img2> num_of_frames min_size_of_seg" << std::endl;
}
