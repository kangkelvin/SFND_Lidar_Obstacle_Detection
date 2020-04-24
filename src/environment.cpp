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
  auto cloud_pair = pcl_processor->SegmentPlane(scanned_cloud, 200, 0.2);
  // renderPointCloud(viewer, cloud_pair.first, "obs",
  //                  Color(1.0, 0.0, 0.0));
  renderPointCloud(viewer, cloud_pair.second, "plane", Color(0.0, 1.0, 0.0));
  auto segmented_obs_clouds =
      pcl_processor->Clustering(cloud_pair.first, 1.0, 3, 30);

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
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI =
      new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud =
      pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud =
      pointProcessorI->FilterCloud(inputCloud, 0.2,
                                   Eigen::Vector4f(0.5, 0.5, 0.5, 1.0),
                                   Eigen::Vector4f(50.0, 50.0, 50.0, 1.0));
  renderPointCloud(viewer, filtered_cloud, "inputCloud");
}

int main(int argc, char** argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // simpleHighway(viewer);
  cityBlock(viewer);

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}