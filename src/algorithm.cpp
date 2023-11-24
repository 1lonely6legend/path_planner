#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D &start,
            Node2D &goal,
            Node2D *nodes2D,
            int width,
            int height,
            CollisionDetection &configurationSpace,
            Visualize &visualization);
void updateH(Node3D &start,
             const Node3D &goal,
             Node2D *nodes2D,
             float *dubinsLookup,
             int width,
             int height,
             CollisionDetection &configurationSpace,
             Visualize &visualization);
Node3D *dubinsShot(Node3D &start, const Node3D &goal, CollisionDetection &configurationSpace);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D *lhs, const Node3D *rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D *lhs, const Node2D *rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
Node3D *Algorithm::hybridAStar(Node3D &start,
                               const Node3D &goal,
                               Node3D *nodes3D,
                               Node2D *nodes2D,
                               int width,
                               int height,
                               CollisionDetection &configurationSpace,
                               float *dubinsLookup,
                               Visualize &visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  // 前驱和后继索引
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  // 可能的方向数，3个用于前进驾驶，另外3个用于倒车
  int dir = Constants::reverse ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  // 算法运行的迭代次数，基于Constants::iterations停止,默认为30000
  int iterations = 0;

  // VISUALIZATION DELAY
  // 可视化延迟
  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  // 使用boost库的二进制堆实现的open list,优先队列
  typedef boost::heap::binomial_heap<Node3D *, boost::heap::compare<CompareNodes>> priorityQueue;
  priorityQueue O;//定义二进制优先堆的名字,是字母o,不是数字0

  // update h value
  // 计算相应点的启发式值,传入的参数有:起始点,目标点,2D节点数组,杜宾查找表,地图宽度,地图高度,碰撞检测类,可视化类
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);
  // mark start as open
  // 标记起始点为open,设置Node3d私有变量o为true,c为false
  start.open();
  // push on priority queue aka open list
  // 将起始点放入优先队列
  O.push(&start);
  //设置前驱索引,返回的是当前节点在3D数组中的索引
  iPred = start.setIdx(width, height);
  //将起始点放入3D数组中,他的索引就是计算的三维在一维数组中的索引
  // 注意区分一下Node3D数组和O数组
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D *nPred;
  Node3D *nSucc;

  // float max = 0.f;

  // continue until O empty
  // 只要优先队列O不为空,就一直循环
  // O中存放的是遍历区域边缘可以发展的节点
  while (!O.empty()) {

    //    // DEBUG
    //    Node3D* pre = nullptr;
    //    Node3D* succ = nullptr;

    //    std::cout << "\t--->>>" << std::endl;

    //    for (priorityQueue::ordered_iterator it = O.ordered_begin(); it != O.ordered_end(); ++it) {
    //      succ = (*it);
    //      std::cout << "VAL"
    //                << " | C:" << succ->getC()
    //                << " | x:" << succ->getX()
    //                << " | y:" << succ->getY()
    //                << " | t:" << helper::toDeg(succ->getT())
    //                << " | i:" << succ->getIdx()
    //                << " | O:" << succ->isOpen()
    //                << " | pred:" << succ->getPred()
    //                << std::endl;

    //      if (pre != nullptr) {

    //        if (pre->getC() > succ->getC()) {
    //          std::cout << "PRE"
    //                    << " | C:" << pre->getC()
    //                    << " | x:" << pre->getX()
    //                    << " | y:" << pre->getY()
    //                    << " | t:" << helper::toDeg(pre->getT())
    //                    << " | i:" << pre->getIdx()
    //                    << " | O:" << pre->isOpen()
    //                    << " | pred:" << pre->getPred()
    //                    << std::endl;
    //          std::cout << "SCC"
    //                    << " | C:" << succ->getC()
    //                    << " | x:" << succ->getX()
    //                    << " | y:" << succ->getY()
    //                    << " | t:" << helper::toDeg(succ->getT())
    //                    << " | i:" << succ->getIdx()
    //                    << " | O:" << succ->isOpen()
    //                    << " | pred:" << succ->getPred()
    //                    << std::endl;

    //          if (pre->getC() - succ->getC() > max) {
    //            max = pre->getC() - succ->getC();
    //          }
    //        }
    //      }

    //      pre = succ;
    //    }

    // pop node with lowest cost from priority queue
    // 弹出优先队列中cost最小的节点,进行下一步的扩展
    nPred = O.top();
    // set index
    // 设置前驱节点的索引
    iPred = nPred->setIdx(width, height);
    iterations++;//增加一次遍历次数,最大遍历次数为30000

    // RViz visualization
    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // 如果当前节点已经被扩展过了,就跳过这个节点,继续下一个节点的扩展
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {//closed表示已经被扩展过了
      // pop node from the open list and start with a fresh node
      // 从优先队列中弹出这个节点,继续下一个节点的扩展
      O.pop();
      continue;
    }
      // _________________
      // EXPANSION OF NODE
      // 如果当前节点没有被扩展过,就进行扩展
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();//首先在Node3D数组中设置为已经扩展过了
      // remove node from open list
      // 将节点从优先队列O中弹出
      O.pop();

      // _________
      // GOAL TEST
      // 查看是否到达了目标点,或者超出了最大遍历次数
      if (*nPred == goal || iterations > Constants::iterations) {
        // DEBUG
        return nPred;//返回当前节点
      }

        // ____________________
        // CONTINUE WITH SEARCH
        // 如果不是目标点,就继续进行搜索
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        /**
         * @brief 如果开启了杜宾曲线,并且当前节点到目标点的距离(包含随机性)小于10,并且当前节点的运动模式小于3
         *
         */
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          //使用杜宾曲线进行搜索
          nSucc = dubinsShot(*nPred, goal, configurationSpace);

          if (nSucc != nullptr && *nSucc == goal) {
            //DEBUG
            // std::cout << "max diff " << max << std::endl;
            return nSucc;
          }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        // 按照前向搜索
        for (int i = 0; i < dir; i++) {
          // 默认情况下dir是6,前后各三个方向
          // create possible successor
          // 根据reedsheeps或者dubin创建可能的下一点，123,456
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          // 计算后继节点的索引
          iSucc = nSucc->setIdx(width, height);

          // ensure successor is on grid and traversable
          // 确保后继节点在地图上，并没有发生碰撞
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            // 确保后继结点没有被便利过，或者后继与前驱一样
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              // 计算新节点的G值，也就是目前走过的代价
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              // 如果该节点被遍历过，或者G值更小
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                // calculate H value
                // 计算未来可能路程
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

                // if the successor is in the same cell but the C value is larger
                // 如果后继节点在相同栅格，但是路径成本更长，删除当前计算的节点，进入下一轮循环
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                // 如果后继结点的走过路径更短，设置前驱是上上个前驱
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }
                //防止陷入循环
                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // put successor on open list
                // 将计算得到的后继结点放入openlist，等待后序遍历
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D &start,
            Node2D &goal,
            Node2D *nodes2D,
            int width,
            int height,
            CollisionDetection &configurationSpace,
            Visualize &visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D *,
                             boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D *nPred;
  Node2D *nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
      // _________________
      // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
        // ____________________
        // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(Node3D &start,
             const Node3D &goal,
             Node2D *nodes2D,
             float *dubinsLookup,
             int width,
             int height,
             CollisionDetection &configurationSpace,
             Visualize &visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins) {
//使用dubin曲线，计算路径成本
    // ONLY FOR dubinsLookup
    //    int uX = std::abs((int)goal.getX() - (int)start.getX());
    //    int uY = std::abs((int)goal.getY() - (int)start.getY());
    //    // if the lookup table flag is set and the vehicle is in the lookup area
    //    if (Constants::dubinsLookup && uX < Constants::dubinsWidth - 1 && uY < Constants::dubinsWidth - 1) {
    //      int X = (int)goal.getX() - (int)start.getX();
    //      int Y = (int)goal.getY() - (int)start.getY();
    //      int h0;
    //      int h1;

    //      // mirror on x axis
    //      if (X >= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);
    //      }
    //      // mirror on y axis
    //      else if (X <= 0 && Y >= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);

    //      }
    //      // mirror on xy axis
    //      else if (X <= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.getT()) / Constants::deltaHeadingRad);

    //      } else {
    //        h0 = (int)(t / Constants::deltaHeadingRad);
    //        h1 = (int)(goal.getT() / Constants::deltaHeadingRad);
    //      }

    //      dubinsCost = dubinsLookup[uX * Constants::dubinsWidth * Constants::headings * Constants::headings
    //                                + uY *  Constants::headings * Constants::headings
    //                                + h0 * Constants::headings
    //                                + h1];
    //    } else {

    /*if (Constants::dubinsShot && std::abs(start.getX() - goal.getX()) >= 10 && std::abs(start.getY() - goal.getY()) >= 10)*/
    //      // start
    //      double q0[] = { start.getX(), start.getY(), start.getT()};
    //      // goal
    //      double q1[] = { goal.getX(), goal.getY(), goal.getT()};
    //      DubinsPath dubinsPath;
    //      dubins_init(q0, q1, Constants::r, &dubinsPath);
    //      dubinsCost = dubins_path_length(&dubinsPath);

    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State *dbStart = (State *) dubinsPath.allocState();
    State *dbEnd = (State *) dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // if reversing is active use a
  if (Constants::reverse && !Constants::dubins) {
    //    ros::Time t0 = ros::Time::now();
    // 使用reedsshepp计算路径成本
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State *rsStart = (State *) reedsSheppPath.allocState();
    State *rsEnd = (State *) reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  // 在2维，地图中计算路径成本
  if (Constants::twoD && !nodes2D[(int) start.getY() * width + (int) start.getX()].isDiscovered()) {
    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    nodes2D[(int) start.getY() * width + (int) start.getX()].setG(aStar(goal2d,
                                                                        start2d,
                                                                        nodes2D,
                                                                        width,
                                                                        height,
                                                                        configurationSpace,
                                                                        visualization));
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }

  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long) start.getX()) - (goal.getX() - (long) goal.getX()))
                          * ((start.getX() - (long) start.getX()) - (goal.getX() - (long) goal.getX())) +
        ((start.getY() - (long) start.getY()) - (goal.getY() - (long) goal.getY()))
            * ((start.getY() - (long) start.getY()) - (goal.getY() - (long) goal.getY())));
    twoDCost = nodes2D[(int) start.getY() * width + (int) start.getX()].getG() - twoDoffset;

  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D *dubinsShot(Node3D &start, const Node3D &goal, CollisionDetection &configurationSpace) {
  // start
  double q0[] = {start.getX(), start.getY(), start.getT()};
  // goal
  double q1[] = {goal.getX(), goal.getY(), goal.getT()};
  // initialize the path
  // 定义dubin曲线
  DubinsPath path;
  // calculate the path
  // 初始化dubin曲线,传入的参数有:起始点,目标点,转弯半径,以及初始化的曲线
  dubins_init(q0, q1, Constants::r, &path);

  //初始化计数器和长度
  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  //根据生成的曲线长度分配节点数组
  Node3D *dubinsNodes = new Node3D[(int) (length / Constants::dubinsStepSize) + 1];

  // avoid duplicate waypoint
  // 避免重复的路径点,开始步长为0.1,每次增加0.1,直到大于曲线长度
  x += Constants::dubinsStepSize;
  while (x < length) {
    double q[3];
    //在曲线上进行采样
    dubins_path_sample(&path, x, q);
    // 设置采样点的x,y,t
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    //进行碰撞检测
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      // 设置当前节点的前驱节点
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        //如果是第一个点,设置前驱为起点
        dubinsNodes[i].setPred(&start);
      }
      //防止出现循环
      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      //如果检测发生碰撞,释放内存,返回空指针
      delete[] dubinsNodes;
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}
