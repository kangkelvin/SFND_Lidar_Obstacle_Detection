/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <cmath>
#include <memory>
#include <unordered_set>
#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  int numDimension;
  std::shared_ptr<Node> left;
  std::shared_ptr<Node> right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL), numDimension(2) {}

  Node(std::vector<float> arr, int setId, int setDim)
      : point(arr), id(setId), left(NULL), right(NULL), numDimension(setDim) {}

  void insert(std::vector<float> new_point, int new_id, int depth) {
    if (new_point[depth % numDimension] < point[depth % numDimension])  // even depth, compare x
      insertIfNull(left, new_point, new_id, depth + 1);
    else
      insertIfNull(right, new_point, new_id, depth + 1);
  }

  void insertIfNull(std::shared_ptr<Node>& node, std::vector<float> new_point,
                    int new_id, int depth) {
    if (node == NULL) {
      node = std::make_shared<Node>(new_point, new_id, numDimension);
    } else {
      node->insert(new_point, new_id, depth);
    }
  }

  void search(std::vector<int>& ids, std::vector<float> target,
              float distanceTol, int depth) {
    int dim_to_check = depth % numDimension;

    bool within_bounding_box = true;
    for (int i = 0; i < numDimension; ++i) {
      within_bounding_box *= point[i] <= target[i] + distanceTol;
      within_bounding_box *= point[i] >= target[i] - distanceTol;
    }

    if (within_bounding_box) {
      if (calcEucledian(target, point) <= distanceTol) {
        ids.push_back(id);
      }
    }

    if (target[dim_to_check] - distanceTol < point[dim_to_check]) {
      searchNullCheck(left, ids, target, distanceTol, depth + 1);
    }
    if (target[dim_to_check] + distanceTol > point[dim_to_check]) {
      searchNullCheck(right, ids, target, distanceTol, depth + 1);
    }
  }

  void searchNullCheck(std::shared_ptr<Node>& node, std::vector<int>& ids,
                       std::vector<float> target, float distanceTol,
                       int depth) {
    if (node == NULL) {
      return;
    } else {
      node->search(ids, target, distanceTol, depth);
    }
  }

  float calcEucledian(std::vector<float> p1, std::vector<float> p2) {
    float result = 0.0;
    for (int i = 0; i < p1.size(); ++i) {
      result += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    }
    return std::sqrt(result);
  }

  void traverseTree() {
    std::cout << "\nHi I am id: " << id << " with: " << point[0] << " "
              << point[1] << " ";
    if (left != NULL && right != NULL) {
      std::cout << "my left side is: " << left->id
                << " and right is: " << right->id << std::endl;
      left->traverseTree();
      right->traverseTree();
    } else if (left != NULL && right == NULL) {
      std::cout << "my left side is: " << left->id << " and right is NULL"
                << std::endl;
      left->traverseTree();
    } else if (left == NULL && right != NULL) {
      std::cout << "my left side is NULL and right is: " << right->id
                << std::endl;
      right->traverseTree();
    } else {
      std::cout << "my left side is NULL and right is NULL" << std::endl;
    }
  }
};

struct KdTree {
  std::shared_ptr<Node> root;

  KdTree() : root(NULL) {}

  void insert(std::vector<float> point, int id, int setDim) {
    if (root == NULL) {
      root = std::make_shared<Node>(point, id, setDim);
      return;
    } else {
      root->insert(point, id, 0);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    root->search(ids, target, distanceTol, 0);
    return ids;
  }

  void proximity(int id, const std::vector<std::vector<float>>& points,
                 std::vector<int>& cluster,
                 std::unordered_set<int>& processed_ids, float distanceTol) {
    processed_ids.insert(id);
    cluster.push_back(id);
    std::vector<int> nearby_points_ids = search(points[id], distanceTol);
    for (auto nearby_points_id : nearby_points_ids) {
      if (processed_ids.find(nearby_points_id) == processed_ids.end()) {
        proximity(nearby_points_id, points, cluster, processed_ids,
                  distanceTol);
      }
    }
  }
};
