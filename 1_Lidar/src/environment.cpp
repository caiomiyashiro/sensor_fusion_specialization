/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "quiz/cluster/kdtree.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    // renderPointCloud(viewer, cloud, "yeah");
    // renderRays(viewer, lidar->position, cloud);

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* ppc = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedClouds = ppc->SegmentPlane(cloud, 1000, 0.2);
    // renderPointCloud(viewer, segmentedClouds.first, "Obstacles", Color(1,0,0));
    // renderPointCloud(viewer, segmentedClouds.second, "Plane", Color(0,1,0));

    // Clusterize and render points
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = ppc->Clustering(segmentedClouds.first, 1.2, 3, 30);
    int i = 0;
    std::vector<Color> color_vector =  {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0), Color(0,1,1), Color(1,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters){
        std::stringstream ss;
        ss << "cluster_" << i;
        renderPointCloud(viewer, cluster, ss.str(), color_vector[i]);

        Box box = ppc->BoundingBox(cluster);
        renderBox(viewer, box, i, color_vector[i], .5);
        i++;
    }
    // Clusterize custom function
  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // sampling and cropping
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud = pointProcessorI->FilterCloud(inputCloud, .2, Eigen::Vector4f (-15, -6, -5, 1), Eigen::Vector4f ( 15, 8, 5, 1));
    // renderPointCloud(viewer,filter_cloud,"filter_cloud");

    // segment into {obstacles and floor}
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedClouds = pointProcessorI->SegmentPlaneCustom(filter_cloud, 200, 0.2);
    // renderPointCloud(viewer, segmentedClouds.first, "Obstacles", Color(1,0,0));
    renderPointCloud(viewer, segmentedClouds.second, "Plane", Color(0,1,0));
    
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> all_points;
    for (int i=0; i<segmentedClouds.first->size(); i++) {
        pcl::PointXYZI pcl_point = segmentedClouds.first->points[i];
        std::vector<float> point = {pcl_point.x, pcl_point.y, pcl_point.z};
    	tree->insert(point, i);
        all_points.push_back(point);
    }

    // euclideanCluster is at kdtree.h (challenges on importing from different files...)
    std::vector<std::vector<int>> clusters = euclideanCluster(all_points, tree, .4);
  	int clusterId = 0;
    std::vector<Color> color_vector =  {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0), Color(0,1,1), Color(1,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster){
      		pcl::PointXYZI temp_point;
      		temp_point.x=all_points[indice][0]; temp_point.y=all_points[indice][1]; temp_point.z=all_points[indice][2]; temp_point.intensity=0;
  			clusterCloud->points.push_back(temp_point);
        }
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),color_vector[clusterId%6]);
		Box box = pointProcessorI->BoundingBox(clusterCloud);
        renderBox(viewer, box, clusterId, color_vector[clusterId%6], .5);
  		++clusterId;
  	}

  	if(clusters.size()==0)
  		renderPointCloud(viewer,inputCloud,"data");

}

void cityBlockPCL(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // sampling and cropping
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud = pointProcessorI->FilterCloud(inputCloud, .2, Eigen::Vector4f (-15, -6, -5, 1), Eigen::Vector4f ( 15, 8, 5, 1));
    // renderPointCloud(viewer,filter_cloud,"filter_cloud");

    // segment into {obstacles and floor}
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedClouds = pointProcessorI->SegmentPlane(filter_cloud, 200, 0.2);
    // renderPointCloud(viewer, segmentedClouds.first, "Obstacles", Color(1,0,0));
    renderPointCloud(viewer, segmentedClouds.second, "Plane", Color(0,1,0));

    // Clusterize and render points - using PCL method
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(segmentedClouds.first, .5, 20, 700);
    int i = 0;
    std::vector<Color> color_vector =  {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0), Color(0,1,1), Color(1,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters){
        std::stringstream ss;
        ss << "cluster_" << i;
        renderPointCloud(viewer, cluster, ss.str(), color_vector[i]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, i, color_vector[i], .5);
        i++;
    }

  	if(clusters.size()==0)
  		renderPointCloud(viewer,inputCloud,"data");

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    // one image test
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // cityBlock(viewer, pointProcessorI, inputCloud);
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 

    // stream image
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped ()){

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);    // custom algorithms 
        // cityBlockPCL(viewer, pointProcessorI, inputCloudI); // using PCL's standard algorithms

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
        }
}