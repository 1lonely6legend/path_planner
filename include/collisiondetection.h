#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {
namespace {
// getConfiguration 函数被定义在一个未命名的命名空间内。这意味着这个函数只能在定义它的源文件中被访问。
// 对于只在一个源文件中使用的帮助函数或实用函数，这是一种很好的封装方法。
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  // 避免2D碰撞检查
  t = 99;
}

inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.
    // 碰撞检测类，确定机器人的给定配置q是否会与环境发生碰撞
   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
    // 它应该返回一个布尔值，如果发生碰撞，则返回true，如果节点安全，则返回false
 */
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection();


  /*!
     \brief evaluates whether the configuration is safe
     评估配置是否安全
     \return true if it is traversable, else false
     如果可遍历，则返回true，否则返回false
  */
  template<typename T> bool isTraversable(const T* node) const {
    /* Depending on the used collision checking mechanism this needs to be adjusted
       根据使用的碰撞检查机制，这需要进行调整
       standard: collision checking using the spatial occupancy enumeration
       标准：使用空间占用枚举进行碰撞检查
       other: collision checking using the 2d costmap and the navigation stack
        其他：使用2d成本图和导航堆栈进行碰撞检查
    */
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    // 为配置分配值
    getConfiguration(node, x, y, t);

    // 2D collision test
    // 2D碰撞测试
    if (t == 99) {
      return !grid->data[node->getIdx()];
    }

    if (true) {
      cost = configurationTest(x, y, t) ? 0 : 1;
    } else {
      cost = configurationCost(x, y, t);
    }

    return cost <= 0;
  }

  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     // 计算机器人在世界W中采用特定配置q的成本
     \param x the x position
     // x位置
     \param y the y position
      // y位置
     \param t the theta angle
      // theta角
     \return the cost of the configuration q of W(q)
      // W(q)的配置q的成本
     \todo needs to be implemented correctly
  */
  float configurationCost(float x, float y, float t) const {return 0;}

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
      // 测试机器人的配置q是否在C_free中
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t) const;

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}

 private:
  /// The occupancy grid
  // 占用栅格
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The collision lookup table
  // 碰撞查找表
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}
#endif // COLLISIONDETECTION_H
