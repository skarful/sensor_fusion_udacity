/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
	
	//To hold results
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	for(int i=0; i<maxIterations; i++){
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;

		//Add random points till we have 3. Note: as a set can only hold unique elements, this will ensure same index is not added again.
		while(inliers.size() < 3){
			inliers.insert(rand()%(cloud->points.size()));
		}

		auto itr = inliers.begin();
		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		float z1 = cloud->points[*itr].z;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
		float z2 = cloud->points[*itr].z;
		itr++;
		float x3 = cloud->points[*itr].x;
		float y3 = cloud->points[*itr].y;
		float z3 = cloud->points[*itr].z;
		
		//Fit the plane
		float a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float d = -1*(a*x1+b*y1+c*z1);

		// Measure distance between every point and fitted line
		for(int j=0; j<cloud->points.size(); j++){

			//Check if j already exists in inliers
			if(inliers.count(j)>0){
				continue;
			}
			float x = cloud->points[j].x;
			float y = cloud->points[j].y;
			float z = cloud->points[j].z;

			float distance = fabs(a*x + b*y + c*z + d)/(sqrt(a*a + b*b + c*c));

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol){
				inliers.insert(j);
			}
		}

		if (inliers.size() > inliersResult.size()){
			std::cout<<"Inliers fitted: "<<inliers.size()<<std::endl;
			inliersResult = inliers;
		}

	}

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	//To hold the result 
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for(int i=0; i<maxIterations; i++){

		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;

		//Add random points till we have 2. Note: as a set can only hold unique elements, this will ensure same index is not added again.
		while(inliers.size() < 2){
			inliers.insert(rand()%(cloud->points.size()));
		}

		auto itr = inliers.begin();
		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
		
		//Fit the line
		float a = y1-y2;
		float b = x2-x1;
		float c = x1*y2 - x2*y1;

		// Measure distance between every point and fitted line
		for(int j=0; j<cloud->points.size(); j++){

			//Check if j already exists in inliers
			if(inliers.count(j)>0){
				continue;
			}
			float x = cloud->points[j].x;
			float y = cloud->points[j].y;

			float distance = fabs(a*x + b*y + c)/(sqrt(a*a + b*b));

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol){
				inliers.insert(j);
			}
		}

		if (inliers.size() > inliersResult.size()){
			inliersResult = inliers;
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
	std::cout<<"Loaded data"<<std::endl;
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 10, 1);
	std::cout<<"Inliers obtained"<<std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	std::cout<<"Trying to render the data"<<std::endl;
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
