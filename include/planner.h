#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.
    为混合A*算法创建接口的类
    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
    它继承自`ros::nav_core::BaseGlobalPlanner`，因此可以轻松地与ROS导航堆栈一起使用（实际上并没有）
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     初始化碰撞以及启发式查找表
     \todo probably removed
  */
  void initializeLookups();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     设置地图，例如通过订阅者的回调函数来监听地图更新
     \param map the map or occupancy grid
       地图或占用栅格
  */
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);

  /*!
     \brief setStart
     \param start the start pose
  */
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
      中央函数入口点，做好准备开始规划
  */
  void plan();

 private:
  /// The node handle
  // 节点句柄
  ros::NodeHandle n;
  /// A publisher publishing the start position for RViz
  // 发布者发布RViz的起始位置
  ros::Publisher pubStart;
  /// A subscriber for receiving map updates
  // 用于接收地图更新的订阅者
  ros::Subscriber subMap;
  /// A subscriber for receiving goal updates
  // 用于接收目标更新的订阅者
  ros::Subscriber subGoal;
  /// A subscriber for receiving start updates
  // 用于接收起始更新的订阅者
  ros::Subscriber subStart;
  /// A listener that awaits transforms
  // 等待转换的监听器
  tf::TransformListener listener;
  /// A transform for moving start positions
  // 用于移动起始位置的转换
  tf::StampedTransform transform;
  /// The path produced by the hybrid A* algorithm
  // 混合A*算法产生的路径
  Path path;
  /// The smoother used for optimizing the path
  // 用于优化路径的平滑器
  Smoother smoother;
  /// The path smoothed and ready for the controller
  // 平滑路径，准备控制器
  Path smoothedPath = Path(true);
  /// The visualization used for search visualization
  // 用于搜索可视化的可视化
  Visualize visualization;
  /// The collission detection for testing specific configurations
  // 用于测试特定配置的碰撞检测
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  // 泰森多边形图
  DynamicVoronoi voronoiDiagram;
  /// A pointer to the grid the planner runs on
  // 指向规划器运行的网格的指针
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  // 通过RViz设置的起始姿态
  geometry_msgs::PoseWithCovarianceStamped start;
  /// The goal pose set through RViz
  // 通过RViz设置的目标姿态
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  // 允许规划器计划的标志
  bool validStart = false;
  /// Flags for allowing the planner to plan
  // 允许规划器计划的标志
  bool validGoal = false;
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  // 车辆配置及其空间占用枚举的查找表
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  /// A lookup of analytical solutions (Dubin's paths)
  // 分析解的查找表（杜宾路径）
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};
}
#endif // PLANNER_H
