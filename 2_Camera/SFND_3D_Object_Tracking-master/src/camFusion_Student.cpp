
#include <iostream>
#include <algorithm>
#include <numeric>
#include <map>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait, bool is_analysis, string fname)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        if(it1->lidarPoints.size() != 0){
            // draw enclosing rectangle
            cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

            // augment object with some key data
            float percentile = .5;
            vector<double> test;
            for(auto it_ = it1->lidarPoints.begin(); it_ != it1->lidarPoints.end(); it_++){
                test.push_back(it_->x);
            }
            sort(test.begin(), test.end());
            int perc = ceil(test.size() * percentile);
            char str1[200], str2[200];
            sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
            putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
            sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", test[perc-1], ywmax-ywmin); //xwmin
            putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
        }
        
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    if(is_analysis){
        cv::imwrite(fname + " Top View.jpg", topviewImg);
    }else{
        // display image
        string windowName = "3D Objects";
        cv::namedWindow(windowName, 1);
        cv::imshow(windowName, topviewImg);

        if(bWait)
        {
            cv::waitKey(0); // wait for key to be pressed
        }
    }
    
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    for(auto it = kptMatches.begin(); it != kptMatches.end(); it++){
        cv::KeyPoint origin_kp, current_kp;
        origin_kp = kptsPrev[it->queryIdx]; 
        current_kp = kptsCurr[it->trainIdx]; 
        if(boundingBox.roi.contains(origin_kp.pt) && boundingBox.roi.contains(current_kp.pt)){
            boundingBox.kptMatches.push_back(*it);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }


    // STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    const int STRATEGY = 1; // 1 - percentiles 2 - average

    vector<double> previous_x, current_x;
    previous_x.resize(lidarPointsPrev.size());
    current_x.resize(lidarPointsCurr.size());
    for(int i=0; i < lidarPointsPrev.size(); i++){
        previous_x[i] = lidarPointsPrev[i].x;
    }
    for(int i=0; i < lidarPointsCurr.size(); i++){
        current_x[i] = lidarPointsCurr[i].x;
    }

    double prev_x_stat, curr_x_stat;
    if(STRATEGY == 1){ // percentile
        const float PERCENTILE = .5;
        int perc_previous = ceil(lidarPointsPrev.size() * PERCENTILE);
        int perc_actual = ceil(lidarPointsCurr.size() * PERCENTILE);
        // partially sort array
        partial_sort(previous_x.begin(), previous_x.begin() + perc_previous, previous_x.end());
        partial_sort(current_x.begin(), current_x.begin() + perc_actual, current_x.end());

        prev_x_stat = previous_x[perc_previous-1];
        curr_x_stat = current_x[perc_actual-1];
    }else if(STRATEGY == 2){ // average
        // check if it's needed
    }

    ///////////////////
    // TTC Calculation/
    ///////////////////
    // first pair of image test on debug - velocity = 0.56 TTC = 14.16
    double velocity = abs(prev_x_stat - curr_x_stat)/(1/frameRate);
    TTC = curr_x_stat/velocity;  
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // for all bboxes, count #commom matches among all other boxes
    map<int,map<int,int>> bbox_match_count; 
    int count = 0;
    for(auto it = matches.begin(); it != matches.end(); it++){
        // find to which bbbox both source and ref match are at
        int previous_point_interest_index = it->queryIdx, current_point_interest_index = it->trainIdx;

        cv::Point2f *previous_p2f = &prevFrame.keypoints[previous_point_interest_index].pt, *current_p2f = &currFrame.keypoints[current_point_interest_index].pt;
        int previous_bbox_ix = -1, current_bbox_ix = -1;
        for(auto it2 = prevFrame.boundingBoxes.begin(); it2 != prevFrame.boundingBoxes.end(); it2++){
            if(it2->roi.contains(*previous_p2f)){
                previous_bbox_ix = it2->boxID;
                break;
            }
        }
        for(auto it2 = currFrame.boundingBoxes.begin(); it2 != currFrame.boundingBoxes.end(); it2++){
            if(it2->roi.contains(*current_p2f)){
                current_bbox_ix = it2->boxID;
                break;
            }
        }
        // only procede if both points of interest are inside a bbox
        if(previous_bbox_ix != -1 && current_bbox_ix != -1){
            // cout << "previous_bbox_ix " << previous_bbox_ix << " current_bbox_ix " << current_bbox_ix << endl;
            if(bbox_match_count.find(previous_bbox_ix) == bbox_match_count.end()){ // key doesn't exist
                map<int,int> bbox_count_relation;
                bbox_count_relation[current_bbox_ix] = 1;
                bbox_match_count[previous_bbox_ix] = bbox_count_relation;
                count++;
            }else{
                // if origin bbox doesn't have source box yet, create map with count 1
                if(bbox_match_count[previous_bbox_ix].find(current_bbox_ix) == bbox_match_count[previous_bbox_ix].end()){
                    bbox_match_count[previous_bbox_ix][current_bbox_ix] = 1;
                    count++;
                }else{ // if origin - source bbox relation already exists, just increment
                    bbox_match_count[previous_bbox_ix][current_bbox_ix]++;
                    count++;
                }
            }
        }
    } // eof all matches pairs. Important output is bbox_match_count

    // for every entry of bbox_match_count, match pair with biggest counts
    for(auto map_it = bbox_match_count.begin(); map_it != bbox_match_count.end(); map_it++){
        int max_count = -1, bbox_id;
        map<int, int> bbox_matches = map_it->second;
        for(auto map_items = bbox_matches.begin(); map_items != bbox_matches.end(); map_items++){
            if(map_items->second > max_count){
                bbox_id = map_items->first;
                max_count = map_items->second;
            }
        }
        bbBestMatches[map_it->first] = bbox_id;
    }

}
