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

  void insert(std::vector<float> new_point, int new_id, bool isEven) {
    if (isEven) {
      if (new_point[0] < point[0])  // even depth, compare x
        insertIfNull(left, new_point, new_id, false);
      else
        insertIfNull(right, new_point, new_id, false);
    } else {
      if (new_point[1] < point[1])  // odd depth, compare y
        insertIfNull(left, new_point, new_id, true);
      else
        insertIfNull(right, new_point, new_id, true);
    }
  }

  void insertIfNull(std::shared_ptr<Node>& node, std::vector<float> new_point,
                    int new_id, bool isEven) {
    if (node == NULL) {
      node = std::make_shared<Node>(new_point, new_id);
    } else {
      node->insert(new_point, new_id, isEven);
    }
  }

  void search(std::vector<int>& ids, std::vector<float> target,
              float distanceTol, int depth) {
    int even_odd_depth = depth % 2;
    bool is_within_upper =
        point[even_odd_depth] < target[even_odd_depth] + distanceTol;
    bool is_within_lower =
        point[even_odd_depth] > target[even_odd_depth] - distanceTol;
    if (is_within_lower && is_within_upper &&
        (calcEucledian(target, point) < distanceTol)) {
      ids.push_back(id);
      searchNullCheck(left, ids, target, distanceTol, depth + 1);
      searchNullCheck(right, ids, target, distanceTol, depth + 1);
    } else if (is_within_upper) {
      searchNullCheck(right, ids, target, distanceTol, depth + 1);
    } else if (is_within_lower) {
      searchNullCheck(left, ids, target, distanceTol, depth + 1);
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
};

struct KdTree {
  std::shared_ptr<Node> root;

  KdTree() : root(NULL) {}

  void insert(std::vector<float> point, int id) {
    if (root == NULL) {
      root = std::make_shared<Node>(point, id);
      return;
    } else {
      root->insert(point, id, true);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    root->search(ids, target, distanceTol, 0);
    return ids;
  }
};
