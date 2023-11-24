#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Each node has a unique configuration (x, y, theta) in the configuration space C.
*/
class Node3D {
 public:

  /// The default constructor for 3D array initialization
  Node3D() : Node3D(0, 0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node3D(float x, float y, float t, float g, float h, const Node3D *pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
  }

  // GETTER METHODS
  /// get the x position
  float getX() const { return x; }
  /// get the y position
  float getY() const { return y; }
  /// get the heading theta
  float getT() const { return t; }
  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  /// get the number associated with the motion primitive of the node
  // 获得与节点的运动原语相关联的数字
  int getPrim() const { return prim; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node3D *getPred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const float &x) { this->x = x; }
  /// set the y position
  void setY(const float &y) { this->y = y; }
  /// set the heading theta
  void setT(const float &t) { this->t = t; }
  /// set the cost-so-far (real value)
  void setG(const float &g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float &h) { this->h = h; }
  /// set and get the index of the node in the 3D grid
  int setIdx(int width, int height) {//计算节点在3D数组中的索引
    this->idx = (int) (t / Constants::deltaHeadingRad)//对转角在弧度上进行离散,默认分为72个角度
        * width * height + (int) (y) * width + (int) (x);//
    //一种常见的三维数组转为一位数组坐标的方法,分成一个个格子个格子里面有72个角度
    //其中(t/Constants::deltaHeadingRad)*width*height代表跳过了t层平面,然后再跳过了y行,最后跳过了x列
    return idx;
  }
  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }
  /// set a pointer to the predecessor of the node
  void setPred(const Node3D *pred) { this->pred = pred; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG();

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator==(const Node3D &rhs) const;

  // RANGE CHECKING
  /// Determines whether it is appropriate to find a analytical solution.
  // 决定是否适合找到分析解
  bool isInRange(const Node3D &goal) const;

  // GRID CHECKING
  /// Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height) const;

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  Node3D *createSuccessor(const int i);

  // CONSTANT VALUES
  /// Number of possible directions
  static const int dir;
  /// Possible movements in the x direction
  static const float dx[];
  /// Possible movements in the y direction
  static const float dy[];
  /// Possible movements regarding heading theta
  static const float dt[];

 private:
  /// the x position
  float x;
  /// the y position
  float y;
  /// the heading theta
  float t;
  /// the cost-so-far
  float g;
  /// the cost-to-go
  float h;
  /// the index of the node in the 3D array
  // 3D数组中节点的索引
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the motion primitive of the node
  int prim;
  /// the predecessor pointer
  const Node3D *pred;
};
}
#endif // NODE3D_H
