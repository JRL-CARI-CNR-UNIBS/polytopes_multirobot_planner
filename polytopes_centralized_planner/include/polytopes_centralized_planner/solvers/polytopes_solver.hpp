#ifndef CVX_SOLVER_H
#define CVX_SOLVER_H

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <graph_core/solvers/tree_solver.h>

#include "polytopes_centralized_planner/polyhedron_container.h"
#include "polytopes_centralized_planner/optimization/formation_problem.h"

#include <polytopes_centralized_planner_msgs/srv/map_with_bounding_boxes.hpp>

#include <iris/iris.h>
#include <iris/geometry.h>

//#include <moveit/planning_scene/planning_scene.h>

//#include <moveit_msgs/srv/get_planning_scene.hpp>

//#include <geometric_shapes/shapes.h>
//#include <geometric_shapes/bodies.h>
//#include <geometric_shapes/body_operations.h>
//#include <geometric_shapes/aabb.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

//#include <rosparam_utilities/rosparam_utilities.h>

namespace pathplan
{
class CVXSolver : public TreeSolver
{
protected:
  virtual bool setProblem(const double &max_time = std::numeric_limits<double>::infinity()); //max_time not used
  std::vector<PolyhedronContainerPtr> poly_list_;
  FormationProblem fproblem_;
  std::multimap<ConnectionPtr, PolyhedronContainerPtr> map_connection_to_poly_;
  PolyhedronContainerPtr goal_poly_;

  Eigen::Matrix2Xd configuration_bounds_;
  Eigen::Matrix3Xd object_vertices_in_obj_;
  Eigen::Matrix3Xd grasping_vertices_in_obj_;
  std::vector<Eigen::Matrix3Xd> robot_vertices_;
  Eigen::Matrix3Xd grasping_vertices_in_robot_;
  std::vector<Eigen::Matrix2d> workspace_bounds_;
//  std::vector<Eigen::Matrix2d> obstacle_vertices_;
  std::map<std::string, std::vector<Eigen::Matrix2d>> obstacles_;
  std::vector<std::string> robot_loaded_;
  std::vector<std::string> base_names_;

//  std::map<NodePtr, NodePtr> reduced_to_full_map;
  std::unordered_map<NodePtr, std::vector<double>> map_node_to_config;

//  planning_scene::PlanningScenePtr planning_scene_;
  std::shared_ptr<rclcpp::Node> node_;


  short int dim_ = 2;
  short int dof_ = 3;

  // TODO: sampler -> sample outside polyhedrons

  bool includeObstacleFromScene();
  bool getPolyhedronFromPoint(const Eigen::VectorXd& c, PolyhedronContainerPtr& return_poly);
  bool getPolyhedronFromNode(const NodePtr& node, PolyhedronContainerPtr& poly, bool attach=true);
  bool getConvexVerticesFromShape(const shapes::ShapeConstPtr& shape, const Eigen::Isometry3d& pose, Eigen::MatrixXd& vertices);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CVXSolver(const MetricsPtr& metrics,
      const CollisionCheckerPtr& checker,
      const SamplerPtr& sampler):
    TreeSolver(metrics, checker, sampler) {}

  void setSampler(const std::shared_ptr<InformedSampler> s) {
    sampler_ = s;
  }

  virtual bool config(const std::shared_ptr<rclcpp::Node>& node) override;
  virtual bool update(PathPtr& solution) override;
  virtual bool update(const Eigen::VectorXd& sample, PathPtr& solution);
  virtual bool addStart(const NodePtr& start_node, const double &max_time = std::numeric_limits<double>::infinity()) override;
  virtual bool addGoal(const NodePtr& goal_node, const double &max_time = std::numeric_limits<double>::infinity()) override;
  virtual bool addStartTree(const TreePtr& start_tree, const double &max_time = std::numeric_limits<double>::infinity()) override;
  virtual void resetProblem() override;

  virtual TreeSolverPtr clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler) override;

  virtual bool update(const NodePtr& n, PathPtr &solution) override
  {
    RCLCPP_FATAL(rclcpp::get_logger("planner"), "Not implemented yet!");
    throw std::runtime_error("");
    return false;
  }

  bool addGoal(const Eigen::Vector3d& sample);

  // Utils
  bool configurationComputeOnly(const Eigen::VectorXd& sample, Eigen::VectorXd& result);
  const std::vector<double> getFormationParams(const NodePtr& node)
  {
    return map_node_to_config[node];
  }
  std::multimap<ConnectionPtr, PolyhedronContainerPtr> getConnectionPolytopesMap()
  {
    return map_connection_to_poly_;
  }

  std::vector<std::string> getRobots()
  {
    return robot_loaded_;
  }

  std::vector<std::string> getRobotBases()
  {
    return base_names_;
  }

  std::map<std::string, std::vector<Eigen::Vector3d>> getConfigFromPath(const PathPtr& path);

private:
  CVXSolver() = delete;
  CVXSolver(CVXSolver&) = delete;
  CVXSolver(CVXSolver&&) = delete;
  CVXSolver operator=(CVXSolver&) = delete;
  CVXSolver operator=(CVXSolver&&) = delete;
};

typedef std::shared_ptr<CVXSolver> CVXSolverPtr;

} // pathplan

#endif // CVX_SOLVER_H
