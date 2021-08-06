// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


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
    
    // Create the voxel based filtering object
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloudFiltered);

    // Create the region of interest based filter
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
    
    // Optional filtering for removing the roof points
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    // iterate through the above created indices
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // for point in indices, push back into the inliers
    for(int point : indices)
        inliers->indices.push_back(point);

    // similar method to seperatepointclouds function when segmenting & creating the obstacle and plane clouds
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    // set the indices as the inliers found above
    extract.setIndices(inliers);
    // set these to negative, because we want to remove them
    extract.setNegative(true);
    // store results in cloudRegion 
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    // Extract the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;

    // TODO:: Fill in this function to find inliers for the cloud.

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    //pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    //pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

// My Ransac method code
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{

    // Time Ransac process
    //auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// While maxIterations is >0
	while(maxIterations--)
	{
		// Randomly pick two points
		std::unordered_set<int> inliers;
		//while inliers <2 insert random point, mod cloud size so the number is between 0 and max cloud size
		while (inliers.size() < 3) //2 for 2d
			inliers.insert(rand()%(cloud->points.size()));

		//2D
		//float x1, y1, x2, y2;

		//3D
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

		// Calculate a, b and c for 2D
		/* 
		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);
		*/

        // For 3D
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
			pcl::PointXYZI point = cloud->points[index];
			// get its x and y values, no z because this is 2D
			float x3 = point.x; //2d
			float y3 = point.y; //2d
			float z3 = point.z; //3d

			// find absolute value (fabs = float absolute)
			//float pointToplane = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b); //2D
			float pointToplane = fabs((a*x3)+(b*y3)+(c*z3)+d)/sqrt((a*a)+(b*b)+(c*c));

			// if distance d is less than or equal to distance tolernce, add to inliers
			//if(d <= distanceTol)
            if(pointToplane <= distanceTol)
				inliers.insert(index);
		}

		// if inliners is greater than the inliers result then just becomes inliers
		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	
	typename pcl::PointCloud<PointT>::Ptr Inliers_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr Outliers_cloud(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZI point = cloud->points[index];
        if(inliersResult.count(index))
            Inliers_cloud->points.push_back(point);
        else
            Outliers_cloud->points.push_back(point);
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> (Outliers_cloud, Inliers_cloud);

    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "Ransac segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    //auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 0.02 = 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    // now do the clustering, iterate through the cluster indices of type pcl::PointIndices
    for(pcl::PointIndices getIndices: clusterIndices)
    {
        // create new cloud cluster
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        // now do the iteration through the indices
        for(int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        
        clusters.push_back(cloudCluster);
    }

    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// My clustering method code
// My Cluster helper
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
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

// My Euclidean cluster
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
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

// My Clustering method using KdTree
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_kd(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    //auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;

    for (int i=0; i < cloud->points.size(); i++) {
        pcl::PointXYZI point = cloud->points[i];
        std::vector<float> pnts;

        // break down point to xyx for the euclidean cluster fnc
        pnts.push_back(point.x);
        pnts.push_back(point.y);
        pnts.push_back(point.z);

        tree->insert(pnts, i);
        points.push_back(pnts);

    }

    // Call the euclideanCluster function for returning a vector of vector ints. Which is a list of cluster indices.
    std::vector<std::vector<int>> clusters_inds = euclideanCluster(points, tree, clusterTolerance);  

    // now do the iteration through the indices
    for(auto cluster_idx : clusters_inds) {
        // create new cloud cluster
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(auto point_idx : cluster_idx) {
            cloudCluster->points.push_back (cloud->points[point_idx]);
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        
        clusters.push_back(cloudCluster);
    }
  	
    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Compute principal directions (centroid, covariance, eigen)
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cluster, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	
	//Transform the original cloud to the origin where the pricipal components corresponds to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
	typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
	pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
	//Get the minimum and maximum points of the transformed cloud.
	PointT minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
	
	BoxQ boxQ;
	
    // Final Transform
	boxQ.bboxQuaternion = eigenVectorsPCA; //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	boxQ.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
	
	boxQ.cube_length = maxPoint.x-minPoint.x;
	boxQ.cube_width = maxPoint.y-minPoint.y;
	boxQ.cube_height = maxPoint.z-minPoint.z;	
	
	return boxQ;
    
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