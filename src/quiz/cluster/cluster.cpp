/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

// This runs recursively, because it's calling itself

void clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol )
{
	// set the indice to true, meaning it's been processed
	processed[indice] = true;
	// push the point back into cluster, so we're just building up the cluster
	cluster.push_back(indice);

	// Let's see which points are near our indice
	// we call tree->search to do this on points[indice] with the distance tolerance
	// to give me a list of indices nearby, I call them, nearest
	std::vector<int> nearest = tree->search(points[indice],distanceTol);
	// Then interate through tat list of nearby indices
	for(int id : nearest)
	{
		// If that given point, hasn't been processed yet
		if(!processed[id])
			// include the unprocessed point into the clusterHelper
			// this time pass it the id, with the other other information along with that cluster
			// so it can slowly build out the cluster, which will give a list of clusters after we've finish the Euclideancluster
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	// vector of vector ints to store the clusters
	std::vector<std::vector<int>> clusters;

	// Create a vector of booleans which are fale which is the length of the number of points.
	// This is to keep track of which points have been processed or not. Points are all the data I hae in my 2D set.
	std::vector<bool> processed(points.size(), false);

	// Now iterate through those 2D points
	int i = 0;
	while( i < points.size())
	{
		// If the point has been processed already, then increment and move onto the next point
		if(processed[i])
		{
			i++;
			continue;
		}
	
		// If the point has not been processed, create a new cluster
		// which is represented by a vector of ints
		std::vector<int> cluster;
		// then use the clusterHelper. Giving it the args, (point id, points, cluster reference, which points have been processed or not,
		// kdTree for the nearby neighbour search, distnace tolerance to accept)
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		//When clustHelper is complete, we receive the cluster, which we can push back into clusters
		clusters.push_back(cluster);
		// Then move onto the next point
		i++;

	}
 
	// Once complete, return clusters
	return clusters;

}

int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);
  
	// test search for searchHelper in kdtree.h
  	std::cout << "Test Search" << std::endl;
  	//std::vector<int> nearby = tree->search({-6,7},3.0);
	std::vector<int> nearby = tree->search({-6,7},3.0);
  	for(int index : nearby)
      std::cout << index << ",";
  	std::cout << std::endl;

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	// Call the euclideanCluster function for returning a vector of vector ints. Which is a list of cluster indices. (ch)
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
