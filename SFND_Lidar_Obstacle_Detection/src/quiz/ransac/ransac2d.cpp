/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <random>
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	int count_iter_max_outliers = 0;
	for(int i=0; i < maxIterations; i++){
		// Randomly sample subset and fit line
		std::vector<pcl::PointXYZ> sample;
		std::sample(cloud->points.begin(), cloud->points.end(), std::back_inserter(sample), 2,  mt19937{std::random_device{}()});
		
		pcl::PointXYZ p1 = sample[0];
		pcl::PointXYZ p2 = sample[1];

		// Line formula Ax + By + C = 0
		float A = p1.y - p2.y;
		float B = p2.x - p1.x;
		float C = (p1.x * p2.y) - (p2.x * p1.y);
		
		// Measure distance between every point and fitted line - ∣Ax+By+C∣/sqrt(A**2+B**2)
		int count_inliers = 0;
		std::unordered_set<int> temp_inliersResult;
		for(int j = 0; j < cloud->points.size(); j++){
			pcl::PointXYZ sample_point = cloud->points[j];
			float x = sample_point.x;
			float y = sample_point.y;
			float sp_distance = std::fabs(A*x + B*y + C)/std::sqrt(A*A + B*B);

			// If distance is smaller than threshold count it as inlier and add index
			if(sp_distance <= distanceTol){
				temp_inliersResult.insert(j);
			}

		}
		std::cout << "temp_inliersResult.size(): " << temp_inliersResult.size() << " - inliersResult.size(): " << inliersResult.size() << std::endl;
		
		if(temp_inliersResult.size() > inliersResult.size()){
			std::cout << "loop " << i << std::endl;
			std:cout << "----------- New Inliers set of size " << temp_inliersResult.size() << std::endl;
			inliersResult = temp_inliersResult;
		}
		
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	int count_iter_max_outliers = 0;
	for(int i=0; i < maxIterations; i++){
		// Randomly sample subset and fit line
		std::vector<pcl::PointXYZ> sample;
		std::sample(cloud->points.begin(), cloud->points.end(), std::back_inserter(sample), 3,  mt19937{std::random_device{}()});
		
		pcl::PointXYZ p1 = sample[0];
		pcl::PointXYZ p2 = sample[1];
		pcl::PointXYZ p3 = sample[2];

		std::vector<float> v1 = {(p2.x - p1.x), (p2.y - p1.y), (p2.z - p1.z)};
		std::vector<float> v2 = {(p3.x - p1.x), (p3.y - p1.y), (p3.z - p1.z)};
		float A = v1[1]*v2[2] - v1[2]*v2[1];
		float B = v1[2]*v2[0] - v1[0]*v2[2];
		float C = v1[0]*v2[1] - v1[1]*v2[0];
		float D = -(A*p1.x + B*p1.y + C*p1.z);
		
		// Measure distance between every point and fitted line - ∣Ax+By+C∣/sqrt(A**2+B**2)
		int count_inliers = 0;
		std::unordered_set<int> temp_inliersResult;
		for(int j = 0; j < cloud->points.size(); j++){
			pcl::PointXYZ sample_point = cloud->points[j];
			float x = sample_point.x;
			float y = sample_point.y;
			float z = sample_point.z;

			float sp_distance = std::fabs(A*x + B*y + C*z + D)/std::sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier and add index
			if(sp_distance <= distanceTol){
				temp_inliersResult.insert(j);
			}

		}
		// std::cout << "temp_inliersResult.size(): " << temp_inliersResult.size() << " - inliersResult.size(): " << inliersResult.size() << std::endl;
		
		if(temp_inliersResult.size() > inliersResult.size()){
			std::cout << "loop " << i << std::endl;
			// std:cout << "----------- New Inliers set of size " << temp_inliersResult.size() << std::endl;
			inliersResult = temp_inliersResult;
		}
		
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, .2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
