/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <cmath>
#include <memory>
#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  std::shared_ptr<Node> left;
  std::shared_ptr<Node> right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}

  void insert(std::vector<float> new_point, int new_id, int depth) {
      if (new_point[depth%2] < point[depth%2])  // even depth, compare x
        insertIfNull(left, new_point, new_id, depth+1);
      else
        insertIfNull(right, new_point, new_id, depth+1);
  }

  void insertIfNull(std::shared_ptr<Node>& node, std::vector<float> new_point,
                    int new_id, int depth) {
    if (node == NULL) {
      node = std::make_shared<Node>(new_point, new_id);
    } else {
      node->insert(new_point, new_id, depth);
    }
  }

  void search(std::vector<int>& ids, std::vector<float> target,
              float distanceTol, int depth) {
    int even_odd_depth = depth % 2;
    bool is_within_upper_x = point[0] <= target[0] + distanceTol;
    bool is_within_lower_x = point[0] >= target[0] - distanceTol;
    bool is_within_upper_y = point[1] <= target[1] + distanceTol;
    bool is_within_lower_y = point[1] >= target[1] - distanceTol;

    if (is_within_lower_x && is_within_upper_x && is_within_lower_y &&
        is_within_upper_y) {
      if (calcEucledian(target, point) <= distanceTol) {
        ids.push_back(id);
      }
    }

    if (target[even_odd_depth] - distanceTol < point[even_odd_depth]) {
      searchNullCheck(left, ids, target, distanceTol, depth+1);
    }
    if (target[even_odd_depth] + distanceTol > point[even_odd_depth]) {
      searchNullCheck(right, ids, target, distanceTol, depth+1);
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
    return std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                     (p1[1] - p2[1]) * (p1[1] - p2[1]));
  }

  void traverseTree() {
    std::cout << "\nHi I am id: " << id << " with: " << point[0] << " " << point[1] << " ";
    if (left != NULL && right != NULL) {
      std::cout << "my left side is: " << left->id << " and right is: " << right->id << std::endl;
      left->traverseTree();
      right->traverseTree();
    } else if (left != NULL && right == NULL) {
      std::cout << "my left side is: " << left->id << " and right is NULL" << std::endl;
      left->traverseTree();
    } else if (left == NULL && right != NULL) {
            std::cout << "my left side is NULL and right is: " << right->id << std::endl;
      right->traverseTree();
    } else {
            std::cout << "my left side is NULL and right is NULL" << std::endl;

    }

  }
};

struct KdTree {
  std::shared_ptr<Node> root;

  KdTree() : root(NULL) {}

  void insert(std::vector<float> point, int id) {
    if (root == NULL) {
      root = std::make_shared<Node>(point, id);
      return;
    } else {
      root->insert(point, id, 0);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    std::cout << "target: " << target[0] << " " << target[1] << std::endl;
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
