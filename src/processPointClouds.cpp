// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
  typename pcl::PointCloud<PointT>::Ptr filtered_cloud(
      new pcl::PointCloud<PointT>);

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(filterRes, filterRes, filterRes);
  voxel_grid.filter(*filtered_cloud);

  pcl::CropBox<PointT> crop_box;
  // filter far distance out
  crop_box.setInputCloud(filtered_cloud);
  crop_box.setMin(minPoint);
  crop_box.setMax(maxPoint);
  crop_box.filter(*filtered_cloud);

  // filter ego points
  crop_box.setInputCloud(filtered_cloud);
  crop_box.setNegative(true);
  crop_box.setMin(Eigen::Vector4f(-1.5, -2, -2, 1));
  crop_box.setMax(Eigen::Vector4f(3.0, 2, 0, 1));
  crop_box.filter(*filtered_cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return filtered_cloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane

  // create containers
  typename pcl::PointCloud<PointT>::Ptr plane_cloud(
      new pcl::PointCloud<PointT>),
      obs_cloud(new pcl::PointCloud<PointT>);

  // copy the points that is part of the plane to plane_cloud, the original
  // cloud is still untouched
  for (int index : inliers->indices) {
    plane_cloud->points.push_back(cloud->points[index]);
  }

  // another pcl magic, this class will filter out points
  pcl::ExtractIndices<PointT> extractor;
  extractor.setInputCloud(cloud);
  extractor.setIndices(inliers);

  // isNegative(true) means whatever is part of the inlier (which is plane_cloud
  // in this case) will be thrown away isNegative(false) means whatever is part
  // of the inlier is kept, and the rest are thrown away BUTTTTT the original
  // cloud is actually untouched, unless you overwrite the container
  extractor.setNegative(true);

  // save that filtered cloud
  extractor.filter(*obs_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obs_cloud, plane_cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  // TODO:: Fill in this function to find inliers for the cloud.

  // container for index of plane points,
  // essentially vector<int>
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  // container for math data on the plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  // the actual magical segmentation class from pcl
  pcl::SACSegmentation<PointT> SACseg;

  SACseg.setOptimizeCoefficients(true);  // optional, good to have
  SACseg.setModelType(pcl::SACMODEL_PLANE);
  SACseg.setMethodType(pcl::SAC_RANSAC);  // we are using the RANSAC algo

  // params on how many iteration it will try, somewhere
  // between 100 to 1000 is good
  SACseg.setMaxIterations(maxIterations);

  // the bigger this is, the more points it will
  // cluster together as a plane, must be sufficiently
  // large
  SACseg.setDistanceThreshold(distanceThreshold);

  // specify input cloud, should the original
  // cloud containing everything
  SACseg.setInputCloud(cloud);

  // actual magic where it will detect planes
  // and put the index of the plane into inliers
  SACseg.segment(*inliers, *coefficients);

  // if the inliers is empty, means no plane detected
  if (inliers->indices.size() == 0) {
    std::cout << "[pcl Segment Plane] no planes found\n";
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  auto segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  // for output of clusters of pointcloud
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // data structure to do point search during clustering, for O(logn)
  // performance
  typename pcl::search::KdTree<PointT>::Ptr kdTree(
      new pcl::search::KdTree<PointT>);
  kdTree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> eucledean_cluster_extractor;
  eucledean_cluster_extractor.setClusterTolerance(clusterTolerance);
  eucledean_cluster_extractor.setMinClusterSize(minSize);
  eucledean_cluster_extractor.setMaxClusterSize(maxSize);
  eucledean_cluster_extractor.setSearchMethod(kdTree);
  eucledean_cluster_extractor.setInputCloud(cloud);

  // actual magic happening, save the indices of clusters
  eucledean_cluster_extractor.extract(cluster_indices);

  for (int i = 0; i < cluster_indices.size(); ++i) {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (int j = 0; j < cluster_indices[i].indices.size(); ++j) {
      cluster->points.push_back(cloud->points[cluster_indices[i].indices[j]]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
    std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
    std::string dataPath) {
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}