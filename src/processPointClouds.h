// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>
#include "render/box.h"

template <typename PointT>
class ProcessPointClouds {
 public:
  // constructor
  ProcessPointClouds();
  // deconstructor
  ~ProcessPointClouds();

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr FilterCloud(
      typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
      Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  SeparateClouds(pcl::PointIndices::Ptr inliers,
                 typename pcl::PointCloud<PointT>::Ptr cloud);

  void calcRansac3DDistwithMultithread(
      typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol,
      std::vector<int> &best_inliers_set, std::mutex &mtx);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  SegmentPlaneWithRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud,
                           int maxIterations, float distanceTol);

  std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>
  ClusteringWithKdTree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       float clusterTolerance, int minSize, int maxSize);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */