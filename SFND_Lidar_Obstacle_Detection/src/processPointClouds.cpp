// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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


//Filtering Function (Using PCL library)
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //Fill in the function to do voxel grid point reduction and region based filtering
    
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_boxed (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_without_car (new pcl::PointCloud<PointT>);

    //Voxel grid
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    //Region of interest
    pcl::CropBox<PointT> crop(true);
    crop.setInputCloud(cloud_filtered);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.filter(*cloud_boxed);

    //Remove car points
    std::vector<int> indices;
    pcl::CropBox<PointT> car(true);
    car.setInputCloud(cloud_boxed);
    car.setMin(Eigen::Vector4f(-2, -2, -1, 1));
    car.setMax(Eigen::Vector4f(2.6, 2, -0.4, 1));
    car.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int index: indices){
        inliers->indices.push_back(index);
    }

    //Subtraction/extraction
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_boxed);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_without_car);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_without_car;

}

//Index extracting to seperate ground plane (PCL library)
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  //Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());

    for(int index: inliers->indices){
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Extract the inliers (subtract)
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true); //Means subtraction
    extract.filter (*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}

//Segmenting ground plane using RANSAC (PCL library)
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointT> seg; //segmentation object. Keeping pointT so it can process any type of cloud
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


//Segmenting ground plane (using custom RANSAC function)
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //Using custom made ransac code
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
            if(distance <= distanceThreshold){
                inliers.insert(j);
            }
        }

        if (inliers.size() > inliersResult.size()){
            // std::cout<<"Inliers fitted: "<<inliers.size()<<std::endl;
            inliersResult = inliers;
        }

    }

    //Convert to PCL indices struct
    pcl::PointIndices::Ptr inliersP (new pcl::PointIndices);
    std::vector<int> indexlist(inliersResult.begin(), inliersResult.end());
    inliersP->indices = indexlist;

    if (inliersP->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersP,cloud);
    return segResult;
}

//Function to help with euclidean clustering
template<typename PointT>
void ProcessPointClouds<PointT>::proximityFunctionCustom(const int id,const std::vector<std::vector<float>>& points, KdTree* tree, std::vector<int>& cluster, std::vector<bool>& processed,const float distanceTol)
{
  // Obtain the point for the current id
  const std::vector<float> point = points[id];
  processed[id] = true;
  cluster.push_back(id);

  // Find nearby ids
  const std::vector<int> nearby_ids = tree->search(point, distanceTol);

  // Iterate over each nearby id
  for (const int query_id : nearby_ids) {
    // If not processed, process it
    if (!processed[query_id]) {
      proximityFunctionCustom(query_id, points, tree, cluster, processed, distanceTol);
    }
  }
}

//Function to help with euclidean clustering using KD tree
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanClusterCustom(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed(points.size(), false);

  // Iterate through each point
  for (size_t i = 0; i < points.size(); i++) {
    if (processed[i]) { continue; }

    std::vector<int> cluster;  // Create cluster
    proximityFunctionCustom(i, points, tree, cluster, processed, distanceTol);
    clusters.push_back(cluster);
  }

  return clusters;
}

//Euclidean clustering using custom kdtree based code
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::customClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    auto startTime = std::chrono::steady_clock::now();

    //Add points to kd tree
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
  
    for (int i=0; i<cloud->points.size(); i++){
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point,i);
        points.push_back(point);
    }

    //Search nearby points
    std::vector<std::vector<int>> clusters = euclideanClusterCustom(points, tree, clusterTolerance);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> returncluster;
    for(std::vector<int> cluster : clusters)
    {
        if(cluster.size() > minSize && cluster.size() < maxSize){
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            for(int indice: cluster){
                clusterCloud->points.push_back(cloud->points[indice]);
            }

            returncluster.push_back(clusterCloud);
        }
        
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return returncluster;
        
}

//Euclidean clustering (using PCL library)
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    //Create point cloud objects from the indices
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : it->indices){
          cloud_cluster->push_back ((*cloud)[idx]); //*
        }
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
        j++;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


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

//=======================
//IO Functions - PCL data
//=======================
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