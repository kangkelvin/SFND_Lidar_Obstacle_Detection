/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <cmath>
#include <mutex>
#include <thread>
#include <unordered_set>
#include "../../processPointClouds.h"
#include "../../render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  int size = 10000;
  for (int i = -1 * size; i < size; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = size;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = size * rx;
    point.y = size * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

void calcRansacDistwithMultithread(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   float distanceTol,
                                   std::unordered_set<int> &best_inliers_set,
                                   std::mutex &mtx) {
  int first_point_idx = rand() % cloud->points.size();
  int second_point_idx = rand() % cloud->points.size();

  if (first_point_idx == second_point_idx) second_point_idx++;

  pcl::PointXYZ first_point = cloud->points[first_point_idx];
  pcl::PointXYZ second_point = cloud->points[second_point_idx];

  double line_A_coeff = first_point.y - second_point.y;
  double line_B_coeff = second_point.x - first_point.x;
  double line_C_coeff =
      first_point.x * second_point.y - second_point.x * first_point.y;

  std::unordered_set<int> inliers_set;

  for (int idx = 0; idx < cloud->points.size(); ++idx) {
    pcl::PointXYZ point = cloud->points[idx];
    double distance =
        std::fabs(line_A_coeff * point.x + line_B_coeff * point.y +
                  line_C_coeff) /
        std::sqrt(line_A_coeff * line_A_coeff + line_B_coeff * line_B_coeff);
    if (distance < distanceTol) {
      inliers_set.insert(idx);
    }
  }
  const std::lock_guard<std::mutex> lck(mtx);
  if (inliers_set.size() > best_inliers_set.size()) {
    best_inliers_set.clear();
    best_inliers_set = std::move(inliers_set);
  }
}

void calcRansacDistNoMultithread(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 float distanceTol,
                                 std::unordered_set<int> &best_inliers_set,
                                 std::mutex &mtx) {
  int first_point_idx = rand() % cloud->points.size();
  int second_point_idx = rand() % cloud->points.size();

  if (first_point_idx == second_point_idx) second_point_idx++;

  pcl::PointXYZ first_point = cloud->points[first_point_idx];
  pcl::PointXYZ second_point = cloud->points[second_point_idx];

  double line_A_coeff = first_point.y - second_point.y;
  double line_B_coeff = second_point.x - first_point.x;
  double line_C_coeff =
      first_point.x * second_point.y - second_point.x * first_point.y;

  std::unordered_set<int> inliers_set;

  for (int idx = 0; idx < cloud->points.size(); ++idx) {
    pcl::PointXYZ point = cloud->points[idx];
    double distance =
        std::fabs(line_A_coeff * point.x + line_B_coeff * point.y +
                  line_C_coeff) /
        std::sqrt(line_A_coeff * line_A_coeff + line_B_coeff * line_B_coeff);
    if (distance < distanceTol) {
      inliers_set.insert(idx);
    }
  }
  const std::lock_guard<std::mutex> lck(mtx);
  if (inliers_set.size() > best_inliers_set.size()) {
    best_inliers_set.clear();
    best_inliers_set = std::move(inliers_set);
  }
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  srand(time(NULL));
  auto startTime = std::chrono::steady_clock::now();

  // TODO: Fill in this function
  std::unordered_set<int> best_inliers_set;
  std::mutex mtx;
  std::vector<std::thread> threads;

  // For max iterations
  for (int it = 0; it < maxIterations; ++it) {
    threads.emplace_back(std::thread(&calcRansacDistwithMultithread, cloud,
                                     distanceTol, std::ref(best_inliers_set),
                                     std::ref(mtx)));
  }

  std::for_each(threads.begin(), threads.end(),
                [](std::thread &t) { t.join(); });

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - startTime);
  std::cout << "calcRansacDistwithMultithread took " << elapsedTime.count()
            << " microseconds" << std::endl;

  best_inliers_set.clear();

  startTime = std::chrono::steady_clock::now();

  for (int it = 0; it < maxIterations; ++it) {
    calcRansacDistNoMultithread(cloud, distanceTol, best_inliers_set, mtx);
  }

  endTime = std::chrono::steady_clock::now();
  elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - startTime);
  std::cout << "calcRansacDistNoMultithread took " << elapsedTime.count()
            << " microseconds" << std::endl;

  return best_inliers_set;
}

int main() {
  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

  // TODO: Change the max iteration and distance tolerance arguments for
  // Ransac function
  std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
