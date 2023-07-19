#ifndef POLYTOPE_H
#define POLYTOPE_H

#include <iris/iris.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/connection.h>
#include <vector>
#include <Eigen/Core>

class PolyhedronContainer
{
protected:
  iris::Polyhedron poly_;
  std::vector<pathplan::NodePtr> nodes_;
  std::vector<pathplan::ConnectionPtr> conn_;
  bool start_flag_ = false;
  bool goal_flag_ = false;

public:
  PolyhedronContainer() {}
  PolyhedronContainer(const iris::Polyhedron poly)
  {
    poly_ = poly;
  }

  void setPolyhedron(const iris::Polyhedron& poly)
  {
    poly_ = poly;
  }

  iris::Polyhedron getPolyhedron()
  {
    return poly_;
  }

  virtual void addConnection(const pathplan::ConnectionPtr& connection)
  {
    conn_.push_back(connection);
  }

  std::vector<pathplan::ConnectionPtr> getConnection()
  {
    return conn_;
  }

  virtual void addNode(const pathplan::NodePtr& node)
  {
    nodes_.push_back(node);
  }

  std::vector<pathplan::NodePtr> getNodes() const
  {
    return nodes_;
  }

  void setFlagStart(bool start)
  {
    start_flag_ = start;
  }

  void setFlagGoal(bool goal)
  {
    goal_flag_ = goal;
  }

  bool getFlagStart()
  {
    return start_flag_;
  }

  bool getFlagGoal()
  {
    return goal_flag_;
  }

};
typedef std::shared_ptr<PolyhedronContainer> PolyhedronContainerPtr;
#endif // POLYTOPE_H
