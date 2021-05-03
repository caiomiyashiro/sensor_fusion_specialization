#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"
#include "dataStructures.h"

using namespace std;

void loadCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT)
{
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

}

void showLidarTopview(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // plot Lidar points into image
    for (auto it_bb = boundingBoxes.begin(); it_bb != boundingBoxes.end(); it_bb++){

        cv::RNG rng(it_bb->boxID);
        cv::Scalar point_color = cv::Scalar(rng.uniform(0,200), rng.uniform(0,200), rng.uniform(0,200));

        if(it_bb->lidarPoints.size() > 0){
            cout << "BOX ID: " << it_bb->boxID << endl;
            float box_top = 1e8, box_left = 1e8, box_bottom = 0, box_right = 0;
            float min_distance = 1e8;
            for (auto it = it_bb->lidarPoints.begin(); it != it_bb->lidarPoints.end(); ++it)
            {
                float xw = (*it).x; // world position in m with x facing forward from sensor
                float yw = (*it).y; // world position in m with y facing left from sensor

                int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
                int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

                // Bounding Box definition
                box_top = y < box_top? y : box_top; 
                box_left = x < box_left? x : box_left;
                box_bottom = y > box_bottom? y : box_bottom;
                box_right = x > box_right? x : box_right;

                // Min Distance
                min_distance = xw < min_distance? xw : min_distance;

                float zw = (*it).z; // world position in m with y facing left from sensor
                if(zw > -1.40){       
                    cv::circle(topviewImg, cv::Point(x, y), 5, point_color, -1);
                }
            }
            char str1[200], str2[200];
            sprintf(str1, "id=%d, #pts=%d", it_bb->boxID, (int)it_bb->lidarPoints.size());
            cv::putText(topviewImg, str1, cv::Point2f(box_left-200, box_bottom+50), cv::FONT_ITALIC, 1, point_color);
            sprintf(str2, "xmin=%2.2f, objsize=%2.2f", min_distance, (box_right - box_left));
            cv::putText(topviewImg, str2, cv::Point2f(box_left-200, box_bottom+125), cv::FONT_ITALIC, 1, point_color);
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

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}


void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints)
{
    // store calibration data in OpenCV matrices
    cv::Mat P_rect_xx(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_xx(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    loadCalibrationData(P_rect_xx, R_rect_xx, RT);

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
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        double shrinkFactor = 0.10;
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
        if(enclosingBoxes.size() == 1){
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

int main()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C53A3_currLidarPts.dat", lidarPoints);

    std::vector<BoundingBox> boundingBoxes;
    readBoundingBoxes("../dat/C53A3_currBoundingBoxes.dat", boundingBoxes);

    clusterLidarWithROI(boundingBoxes, lidarPoints);
    showLidarTopview(boundingBoxes, cv::Size(10.0, 25.0), cv::Size(1000, 2000));

    return 0;
}