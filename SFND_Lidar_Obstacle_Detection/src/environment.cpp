/* \author Aaron Brown, modified by Sukumar */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
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
    bool renderScene = false;   //To toggle simulation view
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    //Create lidar sensor
    double gSlope = 0;
    Lidar *lidar = new Lidar(cars, gSlope);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);       //To see rays on simulation
    // renderPointCloud(viewer, cloud, "Pcloud");      //To see pointcloud output
    std::cout<<"Lidar ready" <<std::endl;

    //Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::cout<<"Point processor created" <<std::endl;
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(cloud, 100, 0.2);
    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));    //To view objects
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));  //To view groundplane

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointProcessor.numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          Box box = pointProcessor.BoundingBox(cluster);
          renderBox(viewer,box,clusterId);
          ++clusterId;
    }
}

//To work on real pcl data
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

    //Step 1: Filter the data using region of interest, voxelgrid
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor->FilterCloud(inputCloud, 0.25f , Eigen::Vector4f (-10, -6, -2, 1), Eigen::Vector4f ( 20, 7, 1, 1));
    
    //Step 2: Segment the ground plane using custom ransac function
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->Segment(filterCloud, 75, 0.2);
    renderPointCloud(viewer, segmentCloud.second,"groundPlane", Color(0,1,0));  //Render ground plane in green

    //Step 3: Euclidean clustering using custom made KD tree functions
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->customClustering(segmentCloud.first, 0.4, 35, 500);

    //Step 4: Visualization
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointProcessor->numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          Box box = pointProcessor->BoundingBox(cluster);
          renderBox(viewer,box,clusterId);
          ++clusterId;
    }
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
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    // Highway simulation for tutorials
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }
}