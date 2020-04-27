/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <memory>

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr& viewer) {
  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // TODO:: Create lidar sensor
  double setGroundSlope = 0.0;
  auto lidar_ptr = std::make_shared<Lidar>(cars, setGroundSlope);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scanned_cloud =
      lidar_ptr->scan();  // simulate lidar point cloud
  // TODO:: Create point processor
  auto pcl_processor = std::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
  auto cloud_pair = pcl_processor->SegmentPlaneWithPcl(scanned_cloud, 200, 0.2);
  // renderPointCloud(viewer, cloud_pair.first, "obs",
  //                  Color(1.0, 0.0, 0.0));
  renderPointCloud(viewer, cloud_pair.second, "plane", Color(0.0, 1.0, 0.0));
  auto segmented_obs_clouds =
      pcl_processor->ClusteringWithPcl(cloud_pair.first, 1.0, 3, 30);

  int clusterId = 0;

  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 1), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : segmented_obs_clouds) {
    std::cout << "cluster size ";
    pcl_processor->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    Box box = pcl_processor->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr& viewer) {
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS) viewer->addCoordinateSystem(1.0);
}

// Real PCD example
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // downsampling of pcd with voxel grid and crop box
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud =
      pointProcessorI->FilterCloud(inputCloud, 0.2,
                                   Eigen::Vector4f(-20, -6.5, -2, 1.0),
                                   Eigen::Vector4f(20, 7.5, 4, 1.0));

  renderPointCloud(viewer, filtered_cloud, "inputCloud");

  // segment out the road plane
  auto segmented_cloud =
      pointProcessorI->SegmentPlaneWithRansac3D(filtered_cloud, 50, 0.3);

  // cluster the obstacles cloud
  auto obstacles_cloud =
      pointProcessorI->ClusteringWithKdTree(segmented_cloud.first, 0.3, 5, 500);

  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 1), Color(0, 0, 1)};
  int clusterId = 0;

  renderPointCloud(viewer, segmented_cloud.second, "road", Color(1, 1, 1));

  // draw bounding box around detected obstacles
  for (auto cluster : obstacles_cloud) {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

int main(int argc, char** argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // simpleHighway(viewer);
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI =
      new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream =
      pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if (streamIterator == stream.end()) streamIterator = stream.begin();

    viewer->spinOnce();
  }
}