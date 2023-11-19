#ifndef CONSTANTS
#define CONSTANTS
/*!
   \file constants.h
   \brief This is a collection of constants that are used throughout the project.
   // 这是整个项目中使用的常量的集合
   \todo All constants need to be checked and documented
*/

////###################################################
////                                               INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//heading => 0-359度，0表示指向正Y的北方
//    X-COORDINATE => designating the width of the grid
//   X坐标=>指定网格的宽度
//    Y-COORDINATE => designating the height of the grid
//   Y坐标=>指定网格的高度

#include <cmath>

/*!
    \brief The namespace that wraps the entire project
    // 包装整个项目的命名空间
    \namespace HybridAStar
*/

namespace HybridAStar {
/*!
    \brief The namespace that wraps constants.h
    // 包装constants.h的命名空间
    \namespace Constants
*/
namespace Constants {
// _________________
// CONFIG FLAGS

/// A flag for additional debugging output via `std::cout`
// 通过`std::cout`的附加调试输出的标志
static const bool coutDEBUG = false;
/// A flag for the mode (true = manual; false = dynamic). Manual for static map or dynamic for dynamic map.
// 模式的标志（true = manual; false = dynamic）。手动用于静态地图或动态用于动态地图。
static const bool manual = true;
/// A flag for the visualization of 3D nodes (true = on; false = off)
// 3D节点可视化的标志（true = on; false = off）
static const bool visualization = false && manual;
/// A flag for the visualization of 2D nodes (true = on; false = off)
// 2D节点可视化的标志（true = on; false = off）
static const bool visualization2D = false && manual;
/// A flag to toggle reversing (true = on; false = off)
// 切换反转的标志（true = on; false = off）
static const bool reverse = true;
/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
// 切换通过Dubin's shot连接路径的标志（true = on; false = off）
static const bool dubinsShot = true;
/// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
// 切换Dubin's启发式的标志，如果启用反转，则应为false（true = on; false = off）
static const bool dubins = false;
/*!
   \var static const bool dubinsLookup
   // dubinsLookup的标志
   \brief A flag to toggle the Dubin's heuristic via lookup, potentially speeding up the search by a lot
   // 通过查找切换Dubin's启发式的标志，可能大大加快搜索速度
   \todo not yet functional
   // 还没有功能
*/
static const bool dubinsLookup = false && dubins;
/// A flag to toggle the 2D heuristic (true = on; false = off)
// 切换2D启发式的标志（true = on; false = off）
static const bool twoD = true;

// _________________
// GENERAL CONSTANTS

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
// 限制算法的最大搜索深度，可能在没有解决方案的情况下终止
static const int iterations = 30000;
/// [m] --- Uniformly adds a padding around the vehicle
// 统一在车辆周围添加填充
static const double bloating = 0;
/// [m] --- The width of the vehicle
// 车辆的宽度
static const double width = 1.75 + 2 * bloating;
/// [m] --- The length of the vehicle
// 车辆的长度
static const double length = 2.65 + 2 * bloating;
/// [m] --- The minimum turning radius of the vehicle
// 车辆的最小转弯半径
static const float r = 6;
/// [m] --- The number of discretizations in heading
// 航向离散化的数量
static const int headings = 72;
/// [°] --- The discretization value of the heading (goal condition)
// 航向的离散化值（目标条件）
static const float deltaHeadingDeg = 360 / (float)headings;
/// [c*M_PI] --- The discretization value of heading (goal condition)
// 航向的离散化值（目标条件）
static const float deltaHeadingRad = 2 * M_PI / (float)headings;
/// [c*M_PI] --- The heading part of the goal condition
// 目标条件的航向部分
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
/// [m] --- The cell size of the 2D grid of the world
// 世界2D网格的单元格大小
static const float cellSize = 1;
/*!
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell
  As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
  This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
  to allow the successor being placed in the same cell.
  // tie breaker在同一单元格中扩展的节点之间打破平局
  // 因为迄今为止的成本大于到来的成本，所以有理由相信算法会更喜欢前任而不是继任者。
  // 这将导致事实上继任者永远不会被任命，而且一个单元格只能扩展一个节点。 tieBreaker人为地增加了前任的成本
  // 允许继任者被任命为同一单元格。
  // 调整因子在同一单元格中扩展的节点之间打破平局，已经走过的成本大于即将到来的成本，所以有理由相信算法会更喜欢前任而不是继任者。
*/
static const float tieBreaker = 0.01;

// ___________________
// HEURISTIC CONSTANTS

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
// 确保具有障碍物的全向启发式的可接受性的因素
static const float factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
// 转弯的运动成本惩罚（选择非直线运动原语）
static const float penaltyTurning = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
// 反向运动成本惩罚（选择运动原语> 2）
static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
// 方向改变的运动成本惩罚（从原语<3更改为原语> 2）
static const float penaltyCOD = 2.0;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
// 当分析解（Dubin's shot）首次触发时，距离目标的距离
static const float dubinsShotDistance = 100;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
// 分析解（Dubin's shot）的步长主要与碰撞检查相关
static const float dubinsStepSize = 1;


// ______________________
// DUBINS LOOKUP SPECIFIC

/// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
static const int dubinsWidth = 15;
/// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
static const int dubinsArea = dubinsWidth * dubinsWidth;


// _________________________
// COLLISION LOOKUP SPECIFIC

/// [m] -- The bounding box size length and width to precompute all possible headings
// 预先计算所有可能的标题的边界框大小长度和宽度
static const int bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);
/// [#] --- The sqrt of the number of discrete positions per cell
// 每个单元格的离散位置数的平方根
static const int positionResolution = 10;
/// [#] --- The number of discrete positions per cell
// 每个单元格的离散位置数
static const int positions = positionResolution * positionResolution;
/// A structure describing the relative position of the occupied cell based on the center of the vehicle
// 描述基于车辆中心的占用单元格的相对位置的结构
struct relPos {
  /// the x position relative to the center
  int x;
  /// the y position relative to the center
  int y;
};
/// A structure capturing the lookup for each theta configuration
// 捕获每个theta配置的查找的结构
struct config {
  /// the number of cells occupied by this configuration of the vehicle
  // 车辆此配置占用的单元格数
  int length;
  /*!
     \var relPos pos[64]
     \brief The maximum number of occupied cells
     // 最大占用单元格数
     \todo needs to be dynamic
  */
  relPos pos[64];
};

// _________________
// SMOOTHER SPECIFIC
/// [m] --- The minimum width of a safe road for the vehicle at hand
// 手边车辆的安全道路的最小宽度
static const float minRoadWidth = 2;

// ____________________________________________
// COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
// 用于可视化目的的颜色定义
/// A structure to express colors in RGB values
// 用RGB值表示颜色的结构
struct color {
  /// the red portion of the color
  float red;
  /// the green portion of the color
  float green;
  /// the blue portion of the color
  float blue;
};
/// A definition for a color used for visualization
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
/// A definition for a color used for visualization
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
/// A definition for a color used for visualization
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
/// A definition for a color used for visualization
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
/// A definition for a color used for visualization
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
}
}

#endif // CONSTANTS

