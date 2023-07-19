#ifndef VARIABLE_SET_H
#define VARIABLE_SET_H

#include <ifopt/variable_set.h>
#include <ifopt/bounds.h>

#include <Eigen/Core>

#include <string>

//#define R_DIMENSION 2 //2 or 3

class FormationVar : public ifopt::VariableSet
{
protected:
  Eigen::VectorXd config_;
  VecBound bounds_;
public:
  FormationVar(const std::string& name,
               const Eigen::VectorXd& x,
               const Eigen::Matrix2Xd& b)
    : ifopt::VariableSet(x.rows(), name)
  {
    if(b.cols() != GetRows())
    {
      throw std::runtime_error("Optimization: variable set: incompatible dimension on bounds:\nbound.cols() = " + std::to_string(b.rows()) + "\nGetRows() = " + std::to_string(GetRows()));
      return;
    }

    config_ = x;
    for (unsigned int idx=0; idx < b.cols(); idx++)
    {
      bounds_.push_back(ifopt::Bounds(b(0,idx), b(1, idx)));
    }
  }

  void SetVariables(const Eigen::VectorXd& x) override
  {
    config_ = x;
  }

  Eigen::VectorXd GetValues() const override
  {
    return config_;
  }

  VecBound GetBounds() const override
  {
    return bounds_;
  }
};

#endif // VARIABLE_SET_H
