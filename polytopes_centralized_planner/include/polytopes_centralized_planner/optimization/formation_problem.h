#ifndef FORMATION_PROBLEM_H
#define FORMATION_PROBLEM_H

#include <vector>
#include <array>

#include <polytopes_centralized_planner/optimization/variable_set.h>
#include <polytopes_centralized_planner/optimization/constraints_set.h>
#include <polytopes_centralized_planner/optimization/cost_term.h>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <iris/iris.h>


// TODO:
// - Change from Matrix to Array each collection of value not linked to linear algebra

class FormationProblem
{
protected:
  const unsigned int space_dim_ = 2;
  const std::string name_varset_ = "var_set";
  const std::string name_constrset_ = "constr_set";
  const std::string name_cost_ = "cost_term";
  std::shared_ptr<ifopt::Problem> problem_;

  Eigen::VectorXd x_init_;
  Eigen::Matrix2Xd variable_bounds_;
  Eigen::VectorXd goal_;

  Eigen::Matrix3Xd vertices_of_obj_in_obj_;
  std::vector<Eigen::Matrix3Xd> vertices_of_robot_in_robot_;
  Eigen::Matrix3Xd grasp_in_obj_;
  std::vector<Eigen::Matrix2Xd> obstacles_;
  Eigen::Matrix3Xd center_to_grapsing_in_robot_;

//  std::shared_ptr<FormationVar> variable_set_ptr_;
//  std::shared_ptr<FormationConstr> contraints_set_ptr_;
//  std::shared_ptr<FormationCost> cost_term_ptr_;
  iris::Polyhedron poly_;

  ifopt::IpoptSolver ipopt_;

public:
  FormationProblem()
  {
    problem_ = std::make_shared<ifopt::Problem>();
    ipopt_.SetOption("jacobian_approximation","finite-difference-values"); // TODO: after Jacobian has been added, change in "exact"
    ipopt_.SetOption("linear_solver","mumps"); //?
  }
  virtual void newProblem()
  {
    problem_ = std::make_shared<ifopt::Problem>();
  }

//  virtual void newProblem(const Eigen::VectorXd& x_init, const Eigen::Vector2d& goal)
//  {
//    newProblem();
//    setVariables(x_init, variable_bounds_);
//    setGoal(goal);
//  }

  void setVariables(const Eigen::VectorXd& x_init, const Eigen::Matrix2Xd& bounds);
  void setVariables(const Eigen::VectorXd& x_init);
  void setBounds(const Eigen::Matrix2Xd& bounds);
  void setGoal(const Eigen::Vector2d& goal)
  {
    goal_ = goal;
  }

  void setObjVertices(const Eigen::Matrix3Xd& vertices_of_obj_in_obj, const Eigen::Matrix3Xd& grasp_in_obj)
  {
    vertices_of_obj_in_obj_ = vertices_of_obj_in_obj;
    grasp_in_obj_ = grasp_in_obj;
  }

  void setRobotsVertices(const std::vector<Eigen::Matrix3Xd>& vertices_of_robot_in_robot, const Eigen::Matrix3Xd center_to_grapsing_in_robot)
  {
    vertices_of_robot_in_robot_ = vertices_of_robot_in_robot;
    center_to_grapsing_in_robot_ = center_to_grapsing_in_robot;
  }

  void setObstacles(const std::vector<Eigen::Matrix2Xd>& obstacles)
  {
    obstacles_ = obstacles;
  }

  void setObstacles(const std::map<std::string, std::vector<Eigen::Matrix2d>>& obstacles)
  {
    obstacles_.clear();
    for(std::pair<std::string, std::vector<Eigen::Matrix2d>> ob: obstacles)
    {
      for(Eigen::Matrix2d m : ob.second)
      {
        obstacles_.push_back(m);
      }
    }
  }

  bool computePolyhedron();

  void setPolyhedron(const iris::Polyhedron& p)
  {
    assert(p.getDimension() == space_dim_);
    poly_ = p;
  }

  iris::Polyhedron getPolyhedron()
  {
    return poly_;
  }

  Eigen::Array2Xd getBounds()
  {
    return variable_bounds_.array();
  }

  bool solve(Eigen::VectorXd& solution, bool print_problem = false);

};

#endif // FORMATION_PROBLEM_H
