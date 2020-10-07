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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	while(maxIterations>0){
		
	// Randomly sample subset and fit line
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

	// Measure distance between every point and fitted line
	for(int i=0;i<cloud->points.size();i++){

		if(inliers.count(i)>0)
			continue;
		
		float x4 = cloud->points[i].x;
		float y4 = cloud->points[i].y;
		float z4 = cloud->points[i].z;

		float distance = fabs(a*x4 + b*y4 +c*z4+d)/sqrt(a*a + b*b + c*c);

		// If distance is smaller than threshold count it as inlier
		if(distance <= distanceTol)
			inliers.insert(i);


	}

	if(inliers.size()>inliersResult.size())
		inliersResult = inliers ;
	

	// Return indicies of inliers from fitted line with most inliers
		
		maxIterations --;
	}

	
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.3);

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
