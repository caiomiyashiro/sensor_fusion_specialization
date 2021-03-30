// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <unordered_set>
// #include "quiz/ransac/ransac2d.cpp"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Voxel dowwsampling
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr box_filtered (new pcl::PointCloud<PointT>);
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    // Cropbox filtering
    typename pcl::CropBox<PointT> boxFilter(true);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloud_filtered);
    boxFilter.filter(*box_filtered);

    // removing EGO ceiling
    typename pcl::CropBox<PointT> ceiling_crop(true);
    std::vector<int> ceiling_indexes;
    ceiling_crop.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    ceiling_crop.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    ceiling_crop.setInputCloud(box_filtered);
    ceiling_crop.filter(ceiling_indexes); // get indices

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int ix : ceiling_indexes)
        inliers->indices.push_back((ix));
    
    typename pcl::ExtractIndices<PointT> remove_ceiling;
    remove_ceiling.setInputCloud(box_filtered);
    remove_ceiling.setIndices (inliers);
    remove_ceiling.setNegative (true);
    remove_ceiling.filter (*box_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return box_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacles (new pcl::PointCloud<PointT>());
    
    pcl::ExtractIndices<PointT> filter;
    filter.setInputCloud(cloud);
    filter.setIndices(inliers);
    filter.filter (*cloud_plane);

    filter.setNegative(true); //instead of getting plane, get obstacles
    filter.filter (*cloud_obstacles);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacles, cloud_plane);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        // PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	int count_iter_max_outliers = 0;
	for(int i=0; i < maxIterations; i++){
		// Randomly sample subset and fit line
		typename std::vector<PointT> sample;
		std::sample(cloud->points.begin(), cloud->points.end(), std::back_inserter(sample), 3,  std::mt19937{std::random_device{}()});
		
		PointT p1 = sample[0];
		PointT p2 = sample[1];
		PointT p3 = sample[2];

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
			pcl::PointXYZI sample_point = cloud->points[j];
			float x = sample_point.x;
			float y = sample_point.y;
			float z = sample_point.z;

			float sp_distance = std::fabs(A*x + B*y + C*z + D)/std::sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier and add index
			if(sp_distance <= distanceThreshold){
				temp_inliersResult.insert(j);
			}

		}
		// std::cout << "temp_inliersResult.size(): " << temp_inliersResult.size() << " - inliersResult.size(): " << inliersResult.size() << std::endl;
		
		if(temp_inliersResult.size() > inliersResult.size()){
			// std::cout << "loop " << i << std::endl;
			// std:cout << "----------- New Inliers set of size " << temp_inliersResult.size() << std::endl;
			inliersResult = temp_inliersResult;
		}
		
	}
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << " Ransac Took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : it->indices)
            cloud_cluster->push_back(cloud->points[idx]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        clusters.push_back(cloud_cluster);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// template<typename PointT>
// std::vector<std::vector<int>>::ClusteringCustom(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol){
    

    

// }


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}