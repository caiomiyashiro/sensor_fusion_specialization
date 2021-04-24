#include <iostream>
#include <numeric>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
  	// create filter kernel
    // apply smoothing
    cv::Mat result = imgGray.clone();
    int filterSize = 5;
    int stdDev = 2.0;
    cv::GaussianBlur(imgGray, result, cv::Size(filterSize, filterSize), stdDev);

    // create filter kernels using the cv::Mat datatype both for x and y
    // ToDo : Add your code here
  	float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2, 
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);
    cv::Mat result_x;
    cv::filter2D(result, result_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  
  	// ------ Y
    float sobel_y[9] = {-1, -2, -1,
                        0, 0, 0, 
                        +1, +2, +1};
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);
    cv::Mat result_y;
    cv::filter2D(result, result_y, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // compute magnitude image based on the equation presented in the lesson 
  	// compute magnitude image
    cv::Mat magnitude = imgGray.clone();
    for (int r = 0; r < magnitude.rows; r++)
    {
        for (int c = 0; c < magnitude.cols; c++)
        {
            magnitude.at<unsigned char>(r, c) = sqrt(pow(result_x.at<unsigned char>(r, c), 2) +
                                                     pow(result_y.at<unsigned char>(r, c), 2));
        }
    }
    
//     // show result
//     string windowNamex = "X Filter";
//     cv::namedWindow(windowNamex, 1); // create window
//     cv::imshow(windowNamex, result_x);
//     cv::waitKey(0);
  
//   	// show result
//     string windowNamey = "Y Filter";
//     cv::namedWindow(windowNamey, 1); // create window
//     cv::imshow(windowNamey, result_y);
//     cv::waitKey(0);
  
  
    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}