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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT> ());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT> ());

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ());
    for(int index:inliers->indices){
        planeCloud->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud,planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers , *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for(pcl::PointIndices getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true ;
        clusters.push_back(cloudCluster);

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

// Project - My Functions Begin

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	

	// For max iterations 
	while(maxIterations>0){
		
	// Randomly sample subset and fit plane
	int i_1 = (rand()%(cloud->points.size()));
	int i_2 = (rand()%(cloud->points.size()));
	int i_3 = (rand()%(cloud->points.size()));


	float x1 = cloud->points[i_1].x;
	float y1 = cloud->points[i_1].y;
	float z1 = cloud->points[i_1].z;
	float x2 = cloud->points[i_2].x;
	float y2 = cloud->points[i_2].y;
	float z2 = cloud->points[i_2].z;
	float x3 = cloud->points[i_3].x;
	float y3 = cloud->points[i_3].y;
	float z3 = cloud->points[i_3].z;

	float a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
	float b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
	float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
	float d = -1*(a*x1+b*y1+c*z1);
	std::unordered_set<int> inliers;
	inliers.insert(i_1);
	inliers.insert(i_2);
    inliers.insert(i_3);
    float denominator =sqrt(a*a + b*b + c*c);
    
	// Measure distance between every point and fitted plane
	for(int i=0;i<cloud->points.size();i++){

		if(inliers.count(i)>0)
			continue;
		
		float x4 = cloud->points[i].x;
		float y4 = cloud->points[i].y;
		float z4 = cloud->points[i].z;

		float distance = fabs(a*x4 + b*y4 +c*z4+d)/denominator;

		// If distance is smaller than threshold count it as inlier
		if(distance <= distanceThreshold)
			inliers.insert(i);


	}

	if(inliers.size()>inliersResult.size())
		inliersResult = inliers ;
	

	    // Return indicies of inliers from fitted line with most inliers
		
		maxIterations --;
	}
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT> ());

	

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers,cloudInliers);
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int index ,const std::vector<std::vector<float>> points , std::vector<int>& cluster,std::vector<bool>& processed,KdTree* tree,float distanceTol)
{
	processed[index] = true ;
	cluster.push_back(index);

	std::vector<int> nearest = tree->search(points[index],distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
			clusterHelper(id,points,cluster,processed,tree,distanceTol);
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::my_clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    
    
    // STEP 1 : Form a Tree
    KdTree* tree = new KdTree;
    
    std::vector<std::vector<float>> points;

    for(int i=0;i<cloud->points.size();i++)
    {   std::vector<float> p = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
    	tree->insert(p,i); 
        points.push_back(p);
    }
    

    //STEP 2 : Eucledian Clustering Alogrithm
	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(cloud->points.size(),false);
	
	for(int i=0;i<cloud->points.size();i++)
	{
		if (processed[i])
		{
			i++;
			continue;
		}
		
		std::vector<int> cluster;
		clusterHelper(i ,points,cluster ,processed,tree ,clusterTolerance);
        if(cluster.size()<minSize || cluster.size()>maxSize)
            continue;
		clusters.push_back(cluster);
		
	}
 

    

    //STEP 3 : Form Point Cloud of the Clusters and return
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cluster_point_clouds ;
    int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{   
        
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int indice: cluster)
        {
  			clusterCloud->points.push_back(cloud->points[indice]);
            
        }
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true ;
        cluster_point_clouds.push_back(clusterCloud);
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return cluster_point_clouds;
}

