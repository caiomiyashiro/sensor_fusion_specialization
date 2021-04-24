#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double max_intersect_perc = 0.0;
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);

    // TODO: Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    vector<cv::KeyPoint> points_of_interest;
    for(int r=0; r<dst_norm.rows; r++){
        for(int c=0; c<dst_norm.cols; c++){
            float pixel_response = dst_norm.at<float>(r,c);
            if(pixel_response > minResponse){
                cv::KeyPoint possible_keypoint;
                possible_keypoint.pt = cv::Point2f(c,r); // fucking point invert the coordinates
                possible_keypoint.size = 2 * apertureSize;
                possible_keypoint.response = pixel_response;

                // for each of the previously found points of interest, check if intersect and,
                // if response if bigger, replace by the pixel with bigger response
                bool has_intersected = false;
                for(auto it=points_of_interest.begin(); it != points_of_interest.end(); it++){
                    float intersec_perc = cv::KeyPoint::overlap(*it, possible_keypoint);
                    if(intersec_perc > max_intersect_perc){
                        has_intersected = true;
                        if(possible_keypoint.response > it->response){
                            *it = possible_keypoint;
                            break;
                        }
                    }
                }

                // if it didn't intersect with anything, just add it to the list of points of interest
                if(!has_intersected){
                    points_of_interest.push_back(possible_keypoint);
                }
            }
        }
    }
    // visualize results
    windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName, 5);
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, points_of_interest, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, visImage);
    cv::waitKey(0);

}

int main()
{
    cornernessHarris();
}