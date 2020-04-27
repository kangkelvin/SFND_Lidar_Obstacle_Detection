// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "quiz/cluster/kdtree.h"

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

template <typename PointT>
void ProcessPointClouds<PointT>::calcRansac3DDistwithMultithread(
    typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol,
    std::vector<int> &best_inliers_set, std::mutex &mtx) {
  std::vector<int> inliers_set;

  // randomly select 3 unique points
  while (inliers_set.size() < 3) {
    int rand_idx = rand() % cloud->points.size();
    inliers_set.push_back(rand_idx);
  }

  // push those 3 random points to vector of PointT
  std::vector<PointT> pts;
  std::vector<int>::iterator it;
  for (it = inliers_set.begin(); it != inliers_set.end(); ++it) {
    pts.push_back(cloud->points[*it]);
  }

  // create vector between p1 p0 and p2 p0
  Eigen::Vector3d v1(pts[1].x - pts[0].x, pts[1].y - pts[0].y,
                     pts[1].z - pts[0].z);
  Eigen::Vector3d v2(pts[2].x - pts[0].x, pts[2].y - pts[0].y,
                     pts[2].z - pts[0].z);

  // calculate plane equation coefficients
  Eigen::Vector3d v1v2 = v1.cross(v2);
  double A = v1v2(0);
  double B = v1v2(1);
  double C = v1v2(2);
  double D = -1 * (A * pts[0].x + B * pts[0].y + C * pts[0].z);

  // check how many points is near this plane
  for (int idx = 0; idx < cloud->points.size(); ++idx) {
    if (std::find(inliers_set.begin(), inliers_set.end(), idx) !=
        inliers_set.end()) {
      PointT p = cloud->points[idx];
      double distance = std::fabs(A * p.x + B * p.y + C * p.z + D) /
                        std::sqrt(A * A + B * B + C * C);
      if (distance < distanceTol) {
        inliers_set.push_back(idx);
      }
    }
  }

  // the plane with most number of points are returned
  const std::lock_guard<std::mutex> lck(mtx);
  if (inliers_set.size() > best_inliers_set.size()) {
    best_inliers_set.clear();
    best_inliers_set = std::move(inliers_set);
  }
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlaneWithRansac3D(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceTol) {
  auto startTime = std::chrono::steady_clock::now();
  srand(time(NULL));

  std::vector<int> best_inliers_set;
  std::mutex mtx;
  std::vector<std::thread> threads;

  // calculate the set of inliers with the most points
  for (int it = 0; it < maxIterations; ++it) {
    threads.emplace_back(std::thread(
        &ProcessPointClouds<PointT>::calcRansac3DDistwithMultithread, this,
        cloud, distanceTol, std::ref(best_inliers_set), std::ref(mtx)));
  }

  std::for_each(threads.begin(), threads.end(),
                [](std::thread &t) { t.join(); });

  // convert unordered_set to PointIndices from pcl
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto it = best_inliers_set.begin(); it != best_inliers_set.end(); it++) {
    inliers->indices.push_back(*it);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  // seperate the road plane from the rest
  auto segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
ProcessPointClouds<PointT>::ClusteringWithKdTree(
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {
  auto startTime = std::chrono::steady_clock::now();

  // for output of clusters of pointcloud
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  KdTree *tree = new KdTree;

  // convert point cloud data to vector<float> for kdTree
  std::vector<std::vector<float>> points;
  for (auto point : cloud->points) {
    std::vector<float> vec_point = {point.x, point.y, point.z};
    points.emplace_back(std::move(vec_point));
  }

  // insert points to the KdTree
  for (int i = 0; i < points.size(); i++)
    tree->insert(points[i], i, points[i].size());

  // to keep track of visited points
  std::unordered_set<int> processed_ids;

  // result of clustering
  std::vector<std::vector<int>> clusters_indices;

  // cluster the points and save in clusters_indices
  for (int id = 0; id < points.size(); id++) {
    if (processed_ids.find(id) == processed_ids.end()) {
      std::vector<int> cluster_indices;
      tree->proximity(id, points, cluster_indices, processed_ids,
                      clusterTolerance);
      if (cluster_indices.size() > minSize &&
          cluster_indices.size() < maxSize) {
        clusters_indices.push_back(cluster_indices);
      }
    }
  }

  // create point cloud from the clusters_indices
  for (int i = 0; i < clusters_indices.size(); ++i) {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (int j = 0; j < clusters_indices[i].size(); ++j) {
      cluster->points.push_back(cloud->points[clusters_indices[i][j]]);
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