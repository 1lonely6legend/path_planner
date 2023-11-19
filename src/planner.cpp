#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // planner类的构造函数，主要是订阅和发布的ros话题
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  // 发布者，发布起点
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // 以下的订阅者，在setMap、setStart、setGoal中，planner初始化之后已经开始监听话题，并交给对应的回调函数处理
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  //使用当前地图更新配置空间
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
  // 为Voronoi图创建数组
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool **binMap;
  binMap = new bool *[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else {
      validStart = false;
    }

    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  // 发布起点，不带协方差给rviz

  //  初始化rviz接收的起点
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan(); }

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr &end) {
  // retrieving goal position
  // 获取目标位置
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan(); }

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  // 如果定义了起点和终点，就开始规划
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    // 数组允许使用 行主顺序的索引方法
    /*为了在这个三维空间（位置和方向）中唯一地定位每个节点，通常会采用一种称为“行主顺序”（row-major order）的索引方法。
     * 这种方法确保每个(x, y, heading)组合都有一个独特的索引。下面是如何实现这一点的：
      三维空间的行主顺序索引：
      假设我们有一个节点的坐标(x, y, heading)。
      每个节点的唯一索引可以计算为：index = heading + depth * (y + height * x)。
      这种计算方法确保了每个不同的(x, y, heading)组合都会映射到一个唯一的索引值。
      为什么这样计算：
      对于每个x（宽度上的位置），我们有height * depth个独特的(y, heading)组合。
      对于每个y（高度上的位置），我们有depth个独特的heading。
      因此，通过上述计算，我们能够确保即使不同的(x, y, heading)组合在乘积上相同，它们在数组中的位置也是唯一的。*/
    // 在三维空间中搜索，每个节点都有一个cost，cost越小，越优先搜索
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    // 定义列表指针并初始化数组
    // 一个三维数组，考虑了航向，每个节点都有一个cost，cost越小，越优先搜索
    Node3D *nodes3D = new Node3D[length]();
    // 二维数组，不考虑方向（除去depth）
    Node2D *nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    // 获取目标位置
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    // 将theta设置为一个值（0,2PI]。角度换弧度
    t = Helper::normalizeHeadingRad(t);
    // 创建一个node3D，作为目标节点
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    // 获取起始位置
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    // 创建一个node3D，作为起始节点
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    // 开始计时
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    // 清除可视化
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    // 找到路径，开始规划
    Node3D *nSolution = Algorithm::hybridAStar(nStart,
                                               nGoal,
                                               nodes3D,
                                               nodes2D,
                                               width,
                                               height,
                                               configurationSpace,
                                               dubinsLookup,
                                               visualization);
    // TRACE THE PATH
    // 追踪路径
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    // 创建更新的路径
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    // 平滑路径
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    // 创建更新的路径
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    // 发布搜索结果
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    delete[] nodes3D;
    delete[] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
