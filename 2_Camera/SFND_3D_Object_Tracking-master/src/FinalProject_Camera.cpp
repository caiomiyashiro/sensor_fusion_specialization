
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

//FP.1 Test function to visualize match points separated by their bounding boxes
void visualize_matches_bbox(vector<DataFrame> &dataBuffer, map<int, int> &bbBestMatches)
{
    DataFrame *current = &(*(dataBuffer.end() - 1)), *previous = &(*(dataBuffer.end() - 2));
    for (auto it_t = bbBestMatches.begin(); it_t != bbBestMatches.end(); it_t++)
    {
        vector<cv::DMatch> kptMatches_;
        for (auto it2 = current->kptMatches.begin(); it2 != current->kptMatches.end(); it2++)
        {
            cv::KeyPoint origin_kp, current_kp;
            origin_kp = previous->keypoints[it2->queryIdx];
            current_kp = current->keypoints[it2->trainIdx];
            if (previous->boundingBoxes[it_t->first].roi.contains(origin_kp.pt) && current->boundingBoxes[it_t->second].roi.contains(current_kp.pt))
            {
                kptMatches_.push_back(*it2);
            }
        }
        cv::Mat matchImg = previous->cameraImg.clone();
        cv::drawMatches(previous->cameraImg, previous->keypoints, current->cameraImg, current->keypoints, kptMatches_,
                        matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        string windowName = "Matches boxes " + to_string(previous->boundingBoxes[it_t->first].boxID) + " and " + to_string(current->boundingBoxes[it_t->second].boxID);
        cv::namedWindow(windowName, 7);
        cv::imshow(windowName, matchImg);
        cv::waitKey(0);
    }
}

ofstream create_output_file_stream()
{
    // Create an output filestream object
    ofstream result;
    //set it write and append mode
    result.open("../dat/analysis.csv");
    return result;
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;  // last file index to load
    int imgStepWidth = 1;
    int imgFillWidth = 4; // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4, 4, cv::DataType<double>::type);        // rotation matrix and translation vector

    RT.at<double>(0, 0) = 7.533745e-03;
    RT.at<double>(0, 1) = -9.999714e-01;
    RT.at<double>(0, 2) = -6.166020e-04;
    RT.at<double>(0, 3) = -4.069766e-03;
    RT.at<double>(1, 0) = 1.480249e-02;
    RT.at<double>(1, 1) = 7.280733e-04;
    RT.at<double>(1, 2) = -9.998902e-01;
    RT.at<double>(1, 3) = -7.631618e-02;
    RT.at<double>(2, 0) = 9.998621e-01;
    RT.at<double>(2, 1) = 7.523790e-03;
    RT.at<double>(2, 2) = 1.480755e-02;
    RT.at<double>(2, 3) = -2.717806e-01;
    RT.at<double>(3, 0) = 0.0;
    RT.at<double>(3, 1) = 0.0;
    RT.at<double>(3, 2) = 0.0;
    RT.at<double>(3, 3) = 1.0;

    R_rect_00.at<double>(0, 0) = 9.999239e-01;
    R_rect_00.at<double>(0, 1) = 9.837760e-03;
    R_rect_00.at<double>(0, 2) = -7.445048e-03;
    R_rect_00.at<double>(0, 3) = 0.0;
    R_rect_00.at<double>(1, 0) = -9.869795e-03;
    R_rect_00.at<double>(1, 1) = 9.999421e-01;
    R_rect_00.at<double>(1, 2) = -4.278459e-03;
    R_rect_00.at<double>(1, 3) = 0.0;
    R_rect_00.at<double>(2, 0) = 7.402527e-03;
    R_rect_00.at<double>(2, 1) = 4.351614e-03;
    R_rect_00.at<double>(2, 2) = 9.999631e-01;
    R_rect_00.at<double>(2, 3) = 0.0;
    R_rect_00.at<double>(3, 0) = 0;
    R_rect_00.at<double>(3, 1) = 0;
    R_rect_00.at<double>(3, 2) = 0;
    R_rect_00.at<double>(3, 3) = 1;

    P_rect_00.at<double>(0, 0) = 7.215377e+02;
    P_rect_00.at<double>(0, 1) = 0.000000e+00;
    P_rect_00.at<double>(0, 2) = 6.095593e+02;
    P_rect_00.at<double>(0, 3) = 0.000000e+00;
    P_rect_00.at<double>(1, 0) = 0.000000e+00;
    P_rect_00.at<double>(1, 1) = 7.215377e+02;
    P_rect_00.at<double>(1, 2) = 1.728540e+02;
    P_rect_00.at<double>(1, 3) = 0.000000e+00;
    P_rect_00.at<double>(2, 0) = 0.000000e+00;
    P_rect_00.at<double>(2, 1) = 0.000000e+00;
    P_rect_00.at<double>(2, 2) = 1.000000e+00;
    P_rect_00.at<double>(2, 3) = 0.000000e+00;

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;                       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer;                 // list of data frames which are held in memory at the same time
    bool bVis = false;                            // visualize results

    ofstream result = create_output_file_stream();
    // Send the various column header names to the created csv file
    result << "detectorType"
           << ";"
           << "descriptorType"
           << ";"
           << "imageIndex"
           << ";"
           << "cameraTTS"
           << endl;
    map<string, vector<vector<string>>> detector_descriptor_matcher_combinations{
        {"SHITOMASI", {{"SIFT"}, {"DES_HOG"}}},
        {"HARRIS", {{"SIFT"}, {"DES_HOG"}}},
        {"FAST", {{"FREAK"}, {"DES_BINARY"}}},
        {"ORB", {{"ORB"}, {"DES_BINARY"}}},
        {"AKAZE", {{"AKAZE"}, {"DES_BINARY"}}},
        {"BRISK", {{"BRISK"}, {"DES_BINARY"}}},
        {"SIFT", {{"SIFT"}, {"DES_HOG"}}}};
    // std::map<string, vector<vector<string>>> detector_descriptor_matcher_combinations{
    //     {"ORB", {{"ORB"}, {"DES_BINARY"}}}
    // };

    for (auto ddm = detector_descriptor_matcher_combinations.begin(); ddm != detector_descriptor_matcher_combinations.end(); ddm++)
    {
        /* MAIN LOOP OVER ALL IMAGES */
        int im_ix = 0; // used to save image names with index
        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex += imgStepWidth)
        {
            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file
            cv::Mat img = cv::imread(imgFullFilename);

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = img;
            dataBuffer.push_back(frame);

            cout << endl
                 << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

            /* DETECT & CLASSIFY OBJECTS */

            float confThreshold = 0.2;
            float nmsThreshold = 0.4;
            detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                          yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);

            cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;

            /* CROP LIDAR POINTS */

            // load 3D Lidar points from file
            string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
            std::vector<LidarPoint> lidarPoints;
            loadLidarFromFile(lidarPoints, lidarFullFilename);

            // remove Lidar points based on distance properties
            float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

            (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

            cout << "#3 : CROP LIDAR POINTS done" << endl;

            /* CLUSTER LIDAR POINT CLOUD */

            // associate Lidar points with camera-based ROI
            float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
            clusterLidarWithROI((dataBuffer.end() - 1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

            // Visualize 3D objects
            bVis = false;
            if (bVis)
            {
                string fname = "NULL";
                show3DObjects((dataBuffer.end() - 1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(1000, 2000), true, false, fname);
            }
            bVis = false;

            cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;

            // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
            // continue; // skips directly to the next image without processing what comes beneath

            /* DETECT IMAGE KEYPOINTS */

            // convert current image to grayscale
            cv::Mat imgGray;
            cv::cvtColor((dataBuffer.end() - 1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

            // extract 2D keypoints from current image
            vector<cv::KeyPoint> keypoints; // create empty feature list for current image
            string detectorType = ddm->first;

            if (detectorType.compare("SHITOMASI") == 0)
            {
                detKeypointsShiTomasi_Harris(keypoints, imgGray, false, false);
            }
            else if (detectorType.compare("HARRIS") == 0)
            {
                detKeypointsShiTomasi_Harris(keypoints, imgGray, true, false);
            }
            else
            {
                vector<string> possible_options = {"FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
                if (std::find(possible_options.begin(), possible_options.end(), detectorType) != possible_options.end())
                {
                    detKeypointsModern(keypoints, imgGray, detectorType, false);
                }
                else
                {
                    throw std::invalid_argument("Invalid parameter 'detectorType' = " + detectorType);
                }
            }

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
                cout << "NOTE: Keypoints have been limited!" << endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (dataBuffer.end() - 1)->keypoints = keypoints;

            cout << "#5 : DETECT KEYPOINTS done" << endl;

            /* EXTRACT KEYPOINT DESCRIPTORS */

            cv::Mat descriptors;
            for (int descriptorType_indv = 0; descriptorType_indv < ddm->second[0].size(); descriptorType_indv++)
            {
                string descriptorType = ddm->second[0][descriptorType_indv]; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

                cout << "#6 : EXTRACT DESCRIPTORS done" << endl;

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";                    // MAT_BF, MAT_FLANN
                    string descriptorTypeMatcher = ddm->second[1][0]; // DES_BINARY, DES_HOG
                    string selectorType = "SEL_KNN";                  // SEL_NN, SEL_KNN

                    cout << endl << "---- Parameters: " <<  detectorType << "; " << descriptorType << "; " << descriptorTypeMatcher << endl;
                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                     (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                     matches, descriptorTypeMatcher, matcherType, selectorType);

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;

                    cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                    /* TRACK 3D OBJECT BOUNDING BOXES */

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
                    map<int, int> bbBestMatches;
                    matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end() - 2), *(dataBuffer.end() - 1)); // associate bounding boxes between current and previous frame using keypoint matches
                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->bbMatches = bbBestMatches;

                    bool flag_visualize_matches_bbox = false;
                    if (flag_visualize_matches_bbox)
                    {
                        visualize_matches_bbox(dataBuffer, bbBestMatches);
                    }

                    cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;

                    /* COMPUTE TTC ON OBJECT IN FRONT */

                    // loop over all BB match pairs
                    for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
                    {
                        // find bounding boxes associates with current match
                        BoundingBox *prevBB, *currBB;
                        for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                        {
                            if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                            {
                                currBB = &(*it2);
                                break;
                            }
                        }

                        for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                        {
                            if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                            {
                                prevBB = &(*it2);
                                break;
                            }
                        }

                        // compute TTC for current match
                        if (currBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0) // only compute TTC if we have Lidar points
                        {
                            //// STUDENT ASSIGNMENT
                            //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                            double ttcLidar;
                            computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                            //// EOF STUDENT ASSIGNMENT

                            //// STUDENT ASSIGNMENT
                            //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                            //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                            double ttcCamera;
                            clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);
                            computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                            //// EOF STUDENT ASSIGNMENT

                            bVis = true;
                            if (bVis)
                            {
                                cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                                showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                                cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);

                                char str[200];
                                sprintf(str, "%d - TTC Lidar - %2.2f s, TTC Camera - %2.2f", im_ix, ttcLidar, ttcCamera);
                                im_ix++;
                                putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));

                                bool is_analysis = true;
                                if (is_analysis)
                                {
                                    string file_name = "../images/analysis_images/" + detectorType + " " + descriptorType + " " + string(str);
                                    cv::imwrite(file_name + ".jpg", visImg);
                                    // just print one loop of the top view lidar points inside bounding boxes
                                    if (detectorType == "SIFT" && descriptorType == "SIFT" && im_ix == 1)
                                    {
                                        show3DObjects((dataBuffer.end() - 1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(1000, 2000), true, is_analysis, file_name);
                                    }
                                    result << detectorType << ";" << descriptorType << ";" << im_ix << ";" << ttcCamera << endl;
                                }
                                else
                                {
                                    string windowName = "Final Results : TTC";
                                    cv::namedWindow(windowName, 4);
                                    cv::imshow(windowName, visImg);
                                    cout << "Press key to continue to next frame" << endl;
                                    cv::waitKey(0);
                                }
                            }
                            bVis = false;

                        } // eof TTC computation
                    }     // eof loop over all BB matches
                }
            } // eof loop over all descriptors

        } // eof loop over all images
        dataBuffer.clear();
    } // eof all combinations detectors/descriptors
    result.close();
    return 0;
}
