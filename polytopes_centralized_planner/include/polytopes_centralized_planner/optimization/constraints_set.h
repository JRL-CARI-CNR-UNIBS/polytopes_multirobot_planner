#ifndef CONSTRAINTS_SET_H
#define CONSTRAINTS_SET_H

#include <ifopt/constraint_set.h>
#include <Eigen/Core>

#include <vector>
#include <memory>

#include <iris/iris.h>
#include <cmath>
#include <limits>

// FOR DEBUG
#include <iostream>



class FormationConstr : public ifopt::ConstraintSet
{
protected:
  /*
   * +-----------+
   * |x1|x2|..|xn|
   * |y1|y2|..|yn|
   * |z1|z2|..|zn|
   * +-----------+
   */
  Eigen::Matrix3Xd vertices_of_obj_in_obj_;
  std::vector<Eigen::Matrix3Xd> vertices_of_robot_in_robot_;
  Eigen::Matrix3Xd center_to_grapsing_in_robot_;
  Eigen::Matrix3Xd grasping_points_in_obj_;
  Eigen::VectorXd fixed_arm_base_angles_;

  // To return
  std::vector<Eigen::Matrix3Xd> vertices_of_robot_in_world_;
  Eigen::Matrix3Xd vertices_of_obj_in_world_;
  bool in_world_computed = false;

  //unsigned int interpolation_k_; // number of interpolation points
  const double contains_tollerance_ = 1E-6;
  unsigned int dim_ = 3;
  unsigned int number_of_manipulators_;
  unsigned int number_of_points_;
  iris::Polyhedron poly_;
  std::string variable_set_name_;

public:
  // size: n_vertex of the object + n_vertex of each robot
  FormationConstr(const std::string& name,
                  const std::string& variable_set_name,
                  const unsigned int number_of_points,
                  const Eigen::Matrix3Xd& vertices_of_obj_in_obj,
                  const std::vector<Eigen::Matrix3Xd>& vertices_of_robot_in_robot,
                  const Eigen::Matrix3Xd& grasping_points_in_obj,
                  const Eigen::Matrix3Xd& center_to_grapsing_in_robot,
                  const iris::Polyhedron& p)
    : ifopt::ConstraintSet(number_of_points, name),
      variable_set_name_(variable_set_name),
      number_of_points_(number_of_points),
      vertices_of_obj_in_obj_(vertices_of_obj_in_obj),
      vertices_of_robot_in_robot_(vertices_of_robot_in_robot),
      grasping_points_in_obj_(grasping_points_in_obj),
      center_to_grapsing_in_robot_(center_to_grapsing_in_robot),
      poly_(p)
  {
    unsigned int points = vertices_of_obj_in_obj.cols();
    for(Eigen::Matrix3Xd m : vertices_of_robot_in_robot)
    {
      points += m.cols();
    }
    assert(points*poly_.getNumberOfConstraints() == number_of_points_);
    // TODO: Tests on vector lengths
    number_of_manipulators_ = vertices_of_robot_in_robot.size();
  }

  Eigen::VectorXd GetValues() const override
  {
    // TODO: DEFINED ONLY FOR R^2, GENERALIZE TO R^3?
    Eigen::VectorXd constr(GetRows());
    //std::cout << "START_CONSTRAINT_GET_VALUES: NUMBER_OF_CONSTRAINTS=" << GetRows() << std::endl;
    // Objects coordinates relative to world
    Eigen::VectorXd config = GetVariables()->GetComponent(variable_set_name_)->GetValues();

    // rotation matrix
    Eigen::Matrix3d rot = Eigen::AngleAxisd(config(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();


    // object's centroid position
    Eigen::Vector3d t;
    t << config(0),
         config(1),
         0;

    unsigned int base = 0; // counter of set of verticies (objects(t_k), manipulators(t_k))
    Eigen::Vector3d vertex_in_world;
    // constraint on the object
    for (unsigned int idx = 0; idx < vertices_of_obj_in_obj_.cols(); idx++)
    {
      vertex_in_world = t + rot * vertices_of_obj_in_obj_.col(idx);
      constr.segment(base, poly_.getNumberOfConstraints()) = poly_.getA() * vertex_in_world.segment<2>(0);
      base += poly_.getNumberOfConstraints();
    }
    Eigen::Matrix3d mrot;
    // for each manipulator
    for (unsigned int idx = 0; idx < vertices_of_robot_in_robot_.size(); idx++)
    {
      mrot = Eigen::AngleAxisd(config(3 + number_of_manipulators_ + idx), Eigen::Vector3d::UnitZ()).toRotationMatrix();
      // constraints on the manipulator
      for (unsigned int jdx = 0; jdx < vertices_of_robot_in_robot_.at(idx).cols(); jdx++)
      {
          vertex_in_world = t + rot * (grasping_points_in_obj_.col(idx) + mrot*(vertices_of_robot_in_robot_.at(idx).col(jdx) - config(3+idx)*center_to_grapsing_in_robot_.col(idx)));
          constr.segment(base, poly_.getNumberOfConstraints()) = poly_.getA() * vertex_in_world.head<2>();
          base += poly_.getNumberOfConstraints();
      }
    }
    return constr;
  }

  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    Eigen::VectorXd B = poly_.getB();
    for(unsigned int idx=0; idx < GetRows(); idx++)
    {
      b.at(idx) = ifopt::Bounds(-std::numeric_limits<double>::infinity(), B(idx % B.rows()));
    }
    return b;
  }

  void FillJacobianBlock(std::string var_set, Jacobian& jac) const override
  {

  }
};

#endif // CONSTRAINTS_SET_H
