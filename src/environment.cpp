/* \author Aaron Brown */
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

// simpleHighway method for simulated pcd
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    //renderscene = false, removes highway and cars
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);

    //CH render with default colour (white)
    //renderPointCloud(viewer, inputCloud,"inputcloud");

    //CH render with specifyin the color (rgb)
    //renderPointCloud(viewer, inputCloud,"inputcloud",Color(1,0,0));

    // TODO:: Create point processor
    // Segment two clouds, one for obstacles and one for ground plane
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // Clustering
    // call point processor clustering function from processpointCloud.cpp file, use the segmentCloud.first which is the obstacle cloud, (distance tolerance, minimum number of points, max number of points)
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    // parameters for default 8 layer sensor
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    // parameters for my 32 layer sensor
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 100);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        //pointProcessor.numPoints(cluster);
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;
    
        // add bounding box to the clustered point
        // box
        bool render_box{ true };
        /*
        if(render_box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }*/
        //boxq
        if(render_box)
        {
            BoxQ boxQ = pointProcessor->BoundingBoxQ(cluster);
            renderBox(viewer,boxQ,clusterId);
        }
        ++clusterId;
    }
}

// cityBlock method for real pcd stored in "../src/sensors/data/pcd/data_1"
// CityBlock for multiple pcd
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ---------------------------------------------------
  // -----  Open 3D viewer and display City Block  -----
  // ---------------------------------------------------

  // Needed for single pcd, this is now being passed innside the main
  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  const pcl::PointCloud<pcl::PointXYZI>::Ptr vox_Cloud{ pointProcessorI->FilterCloud(inputCloud, 0.1f, Eigen::Vector4f(-(30.0 / 2), -6.5, -2.5, 1), Eigen::Vector4f(30.0, 6.5, 2.5, 1)) };

  // Render complete input cloud rom pcd file
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  // Render filtered cloud
  //renderPointCloud(viewer,vox_Cloud,"Voxel & ROI Cloud");
  
  // Segment two clouds, one for obstacles and one for ground plane
  // existing method
  //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(vox_Cloud, 100, 0.2);
  // my ransac method
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->Ransac(vox_Cloud, 100, 0.2); //50, 0.3

  // Rendering of both now segmented clouds
  //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

  // Now do the Clustering
  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500); //0.53, 10, 500
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering_kd(segmentCloud.first, 0.5, 10, 500); 

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
    ++clusterId;

    // Bounding Boxes
    // box
    bool render_box{ true };
    if(render_box)
    {
        // Box class Box for non-quaternion bounding box representation
        Box box = pointProcessorI->BoundingBox(cluster);
        // Box class BoxQ for quarternion bounding box represention
        //BoxQ box = pointProcessorI->BoundingBoxQ(cluster);
        renderBox(viewer,box,clusterId);
    }    
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway method for simulated point cloud data
    //simpleHighway(viewer);

    // cityBlock method for real point cloud data, single pcd file
    //cityBlock(viewer);

    // CityBlock method for multiple pcd files
    // Creat point cloud processor
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    // declare the path where the data lives & begin streaming the files
    std::vector<boost::filesystem::path> stream{ pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1") };
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear and clean all point clouds
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //Load pcd and run obstacle detection process
       inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
       cityBlock(viewer, pointProcessorI, inputCloudI);

       streamIterator++;

       if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}