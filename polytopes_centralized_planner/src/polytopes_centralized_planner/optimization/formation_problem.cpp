#include <polytopes_centralized_planner/optimization/formation_problem.h>

void FormationProblem::setVariables(const Eigen::VectorXd &x_init, const Eigen::Matrix2Xd &bounds)
{
  assert(x_init.rows() == bounds.cols());
  x_init_ = x_init;
  variable_bounds_ = bounds;
}

void FormationProblem::setVariables(const Eigen::VectorXd& x_init)
{
  assert(x_init.rows() == variable_bounds_.cols());
  x_init_ = x_init;
}

void FormationProblem::setBounds(const Eigen::Matrix2Xd& bounds)
{
  variable_bounds_ = bounds;
}

bool FormationProblem::computePolyhedron()
{
  Eigen::VectorXd seed = x_init_.head(space_dim_);

  iris::IRISProblem prb(space_dim_);
  prb.setSeedPoint(seed);
  for (const Eigen::MatrixXd& ob : obstacles_)
  {
    prb.addObstacle(ob);
  }

  iris::IRISOptions opt;
  opt.require_containment = true;
  iris::IRISRegion region = iris::inflate_region(prb, opt);

  poly_ = region.getPolyhedron(); // TODO: Test if is ok
  return true;
}

bool FormationProblem::solve(Eigen::VectorXd &solution, bool print_problem)
{
  problem_->AddVariableSet(std::make_shared<FormationVar>(name_varset_, x_init_, variable_bounds_));
  int number_of_vertices = vertices_of_obj_in_obj_.cols();
  for (auto v_robot : vertices_of_robot_in_robot_)
  {
    number_of_vertices += v_robot.cols();
  }
  problem_->AddConstraintSet(std::make_shared<FormationConstr>(name_constrset_,
                                                              name_varset_,
                                                              number_of_vertices * poly_.getNumberOfConstraints(),
                                                              vertices_of_obj_in_obj_,
                                                              vertices_of_robot_in_robot_,
                                                              grasp_in_obj_,
                                                              center_to_grapsing_in_robot_,
                                                              poly_)
                            );
  problem_->AddCostSet(std::make_shared<FormationCost>(name_cost_, name_varset_, goal_));
  if(print_problem) problem_->PrintCurrent();
  ipopt_.Solve(*problem_);
  if(ipopt_.GetReturnStatus() != 0)
  {
    std::cerr << "There has been a problem during optimization" << std::endl;
    return false;
  }
  solution = problem_->GetOptVariables()->GetValues(); // TODO: And when fails??
  return true;
}
