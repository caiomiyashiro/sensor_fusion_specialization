/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <stdexcept>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "ring_buffer.hpp"
#include "matching2D.hpp"

using namespace std;

// int main(int argc, const char *argv[]){
//     RingBuffer<float> test(2);
//     test.push(1);
//     test.push(2);
//     test.push(3);
//     cout << test << endl;

//     test.push(4);
//     test.push(5);
//     cout << test << endl;
//     return 0;
// }

ofstream create_output_file_stream()
{
    // Create an output filestream object
    ofstream result;
    //set it write and append mode
    result.open("../data/results.csv");
    return result;
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "data/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2; // no. of images which are held in memory (ring buffer) at the same time
    // vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    RingBuffer<DataFrame> dataBuffer(dataBufferSize);
    bool bVis = true; // visualize results

    ofstream result = create_output_file_stream();
    // Send the various column header names to the created csv file
    result << "detectorType"
           << ";"
           << "descriptorType"
           << ";"
           << "detectorTime(ms)"
           << ";"
           << "descriptorTime(ms)"
           << ";"
           << "match_time(ms)"
           << ";"
           << "detectectedKeypoints"
           << ";"
           << "averageKPSize"
           << ";"
           << "matchedKeypoints"
           << ";"
           << "totalTime(ms)"
           << ";" << endl;

    // BRISK: One important point is that BRIEF is a feature descriptor, it doesn't provide any method
    // to find the features. So you will have to use any other feature detectors like SIFT, SURF etc.
    // The paper recommends to use CenSurE which is a fast detector and BRIEF works even slightly better
    // for CenSurE points than for SURF points. https://docs.opencv.org/3.4/dc/d7d/tutorial_py_brief.html
    std::map<string, vector<vector<string>>> detector_descriptor_matcher_combinations{
        {"SHITOMASI", {{"SIFT"}, {"DES_HOG"}}},
        {"HARRIS", {{"SIFT"}, {"DES_HOG"}}},
        {"FAST", {{"BRISK", "BRIEF", "FREAK"}, {"DES_BINARY"}}},
        {"ORB", {{"ORB"}, {"DES_BINARY"}}},
        {"AKAZE", {{"AKAZE"}, {"DES_BINARY"}}},
        {"BRISK", {{"BRISK"}, {"DES_BINARY"}}},
        {"SIFT", {{"BRIEF", "FREAK", "SIFT"}, {"DES_BINARY"}}}
    };

    for (const auto &d_t_m_c : detector_descriptor_matcher_combinations)
    {
        string detectorType = d_t_m_c.first;
        vector<vector<string>> parameters = d_t_m_c.second;
        vector<string> descriptors = parameters[0];
        string matcherType = "MAT_BF";            // MAT_BF, MAT_FLANN
        string descriptorType = parameters[1][0]; // DES_HOG, DES_BINARY
        string selectorType = "SEL_KNN";          // SEL_NN, SEL_KNN

        for (const auto descriptorType : descriptors)
        {
            cout << endl
                 << "====================== detectorType " << detectorType << " descriptorType " << descriptorType << endl;
            /* MAIN LOOP OVER ALL IMAGES */
            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                string test = argv[0];
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
                DataFrame frame;
                frame.cameraImg = imgGray;
                dataBuffer.push(frame);

                //// EOF STUDENT ASSIGNMENT
                cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                double detector_time;
                int nbr_detected_keypoints;
                double avg_kp_size;
                if (detectorType.compare("SHITOMASI") == 0)
                {
                    detKeypointsShiTomasi_Harris(keypoints, detector_time, nbr_detected_keypoints, imgGray, false, false);
                    avg_kp_size = 0;
                }
                else if (detectorType.compare("HARRIS") == 0)
                {
                    detKeypointsShiTomasi_Harris(keypoints, detector_time, nbr_detected_keypoints, imgGray, true, false);
                    avg_kp_size = 0;
                }
                else
                {
                    vector<string> possible_options = {"FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
                    if (std::find(possible_options.begin(), possible_options.end(), detectorType) != possible_options.end())
                    {
                        detKeypointsModern(keypoints, detector_time, nbr_detected_keypoints, avg_kp_size, imgGray, detectorType, false);
                    }
                    else
                    {
                        throw std::invalid_argument("Invalid parameter 'detectorType' = " + detectorType);
                    }
                }
                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                vector<cv::KeyPoint> keypoints_rect; // create empty feature list for current image
                cv::Rect vehicleRect(535, 180, 180, 150);
                if (bFocusOnVehicle)
                {
                    for (int i = 0; i < keypoints.size(); i++)
                    {
                        if (vehicleRect.contains(keypoints[i].pt))
                        {
                            keypoints_rect.push_back(keypoints[i]);
                        }
                    }
                    cout << "#2 : Filtered rectangle. N points from " << keypoints.size() << " to " << keypoints_rect.size() << endl;
                    keypoints = keypoints_rect;
                }

                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType.compare("SHITOMASI") == 0)
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                // (dataBuffer.end() - 1)->keypoints = keypoints; // old
                cout << "#3 : DETECT KEYPOINTS done" << endl; // old
                DataFrame *actual_elem = dataBuffer.get_newest_elem_reference();
                actual_elem->keypoints = keypoints;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;
                double descriptor_time;
                // BRISK, BRIEF, ORB (works with FAST detector), FREAK, AKAZE (works with AKAZE detector), SIFT
                descKeypoints(actual_elem->keypoints, descriptor_time, actual_elem->cameraImg, descriptors, descriptorType);
                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                // (dataBuffer.end() - 1)->descriptors = descriptors;
                actual_elem->descriptors = descriptors;

                cout << "#4 : EXTRACT DESCRIPTORS done" << endl;

                int nbr_matched_keypoints;
                double match_time;
                if (dataBuffer.get_size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */
                    vector<cv::DMatch> matches;

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                    DataFrame *previous_elem = dataBuffer.get_oldest_elem_reference();
                    matchDescriptors(previous_elem->keypoints, actual_elem->keypoints,
                                     previous_elem->descriptors, actual_elem->descriptors,
                                     matches, descriptorType, matcherType, selectorType, match_time);
                    nbr_matched_keypoints = matches.size();

                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    actual_elem->kptMatches = matches;

                    cout << "#5 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                    // visualize matches between current and previous image
                    if (bVis)
                    {
                        cv::Mat matchImg = (actual_elem->cameraImg).clone();
                        cv::drawMatches(previous_elem->cameraImg, previous_elem->keypoints,
                                        actual_elem->cameraImg, actual_elem->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                        string path = "../data/processing_analysis";
                        string file_name = detectorType + "_" + descriptorType + "_" + to_string(imgIndex) + ".jpg";
                        file_name = path + "/" + file_name;
                        cv::imwrite(file_name, matchImg); // A JPG FILE IS BEING SAVED

                        // string windowName = "Matching keypoints between two camera images";
                        // cv::namedWindow(windowName, 7);
                        // cv::imshow(windowName, matchImg);
                        // cout << "Press key to continue to next image" << endl;
                        // cv::waitKey(0); // wait for key to be pressed
                    }
                }
                else
                {
                    nbr_matched_keypoints = 0;
                    match_time = 0;
                }
                double total_time = detector_time + descriptor_time + match_time;
                result << detectorType << ";" << descriptorType << ";" << detector_time << ";" << descriptor_time << ";" << match_time << ";" << nbr_detected_keypoints << ";" << avg_kp_size << ";" << nbr_matched_keypoints << ";" << total_time << ";" << endl;

            } // eof loop over all images

            dataBuffer.clean(); // clean empty dataBuffer for next descriptor types
        }                       // eof descriptorTypes loop
    }                           // eof mapping loop

    result.close();
    return 0;
}