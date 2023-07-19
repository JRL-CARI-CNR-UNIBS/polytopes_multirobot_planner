#ifndef COST_TERM_H
#define COST_TERM_H

#include <ifopt/cost_term.h>

class FormationCost : public ifopt::CostTerm
{
protected:
  std::string varset_name_;
  Eigen::Vector2d goal_;
public:
  FormationCost(const std::string& name,
                const std::string& varset_name,
                const Eigen::Vector2d& goal)
    : ifopt::CostTerm(name), goal_(goal)
  {
    varset_name_ = varset_name;
  }

  double GetCost() const override
  {
    Eigen::VectorXd x = GetVariables()->GetComponent(varset_name_)->GetValues();
    Eigen::Vector2d t = x.head<2>();
    return (t - goal_).squaredNorm();
  }

  void FillJacobianBlock(std::string var_set, Jacobian &jac) const override
  {
    Eigen::VectorXd x = GetVariables()->GetComponent(var_set)->GetValues();
    Eigen::Vector2d t = x.head<2>();
    jac.coeffRef(0,0) = 2*(t(0)-goal_(0));
    jac.coeffRef(0,1) = 2*(t(1)-goal_(1));
    for(unsigned int idx=2; idx < x.rows(); idx++)
    {
      jac.coeffRef(0,idx) = 0;
    }
  }
};

#endif // COST_TERM_H
