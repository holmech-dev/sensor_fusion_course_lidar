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
	//auto StartTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	// While maxIterations is >0
	while(maxIterations--)
	{
		// Randomly pick two points
		std::unordered_set<int> inliers;
		//while inliers <2 insert random point, mod cloud size so the number is between 0 and max cloud size
		while (inliers.size() < 3) //2 for 2d
			inliers.insert(rand()%(cloud->points.size()));

		//2d
		//float x1, y1, x2, y2;

		//3d
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		// Find the first value
		auto itr = inliers.begin();
		// de-reference the pointer to get the value
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z; //for 3d
		// iterate to get the next value
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z; //for 3d
		//iterate to get the next value for 3d
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z; //for 3d

		// Calculate a, b and c for 2d
		/* 
		//2D
		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);
		*/

		//v1 = x2 - x1, y2 - y1, z2 - z1 
		// A = i
		float a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		// B = j
		float b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		// C = k
		float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		// D = -(i*x1+j*y1+k*z1)
		float d = -((a*x1)+(b*y1)+(c*z1));


		// now iterate through the remaining upto the size of the point cloud
		for(int index = 0; index < cloud->points.size(); index++)
		{

			// check if inliers contains an element before continuing
			if(inliers.count(index)>0)
				continue;
			
			// Now calculate the distance and see it its within the threshold
			// grab the pcl point
			pcl::PointXYZ point = cloud->points[index];
			// get its x and y values, no z because this is 2D
			float x3 = point.x; //2d
			float y3 = point.y; //2d
			float z3 = point.z; //3d

			// find absolute value (fabs = float absolute)
			//float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b); //2d
			float pointToplane = fabs((a*x3)+(b*y3)+(c*z3)+d)/sqrt((a*a)+(b*b)+(c*c));

			// if distance d is less than or equal to distance tolernce, add to inliers
			if(d <= distanceTol)
				inliers.insert(index);
		}

		// if inliners is greater than the inliers result then just becomes inliers
		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	
	//auto endtime = std::chrono::steady_clock::now();
	//auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	//std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(); //2d
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(); //3d

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.3);

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
