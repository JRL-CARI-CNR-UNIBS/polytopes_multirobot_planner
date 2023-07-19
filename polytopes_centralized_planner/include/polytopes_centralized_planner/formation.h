#ifndef FORMATION_H
#define FORMATION_H

#include <map>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

/*
 * configuration: [tx, ty, phi, a1, a2, ..., an, theta1, theta2, ..., thetan]'
 */

class Formation
{
private:
protected:
  unsigned int m_rnum;
  std::vector<std::string> m_rnames;

  Eigen::Matrix3Xd                   m_vertices_of_obj_in_obj;
  std::vector<Eigen::Matrix3Xd>  m_vertices_of_robot_in_robot;
  Eigen::Matrix3Xd              m_center_to_grapsing_in_robot;
  Eigen::Matrix3Xd                   m_grasping_points_in_obj;
  std::vector<double> m_base_angles;

  Eigen::VectorXd m_actual_configuration;
  std::map<std::string, Eigen::Isometry3d> m_robot_poses;
  bool is_configured {false};
public:
  Formation(const std::vector<std::string> rnames,
            const Eigen::Matrix3Xd& vertices_of_obj_in_obj                 ,
            const std::vector<Eigen::Matrix3Xd>& vertices_of_robot_in_robot,
            const Eigen::Matrix3Xd& grasping_points_in_obj                 ,
            const Eigen::Matrix3Xd& center_to_grapsing_in_robot
            )
    : m_vertices_of_obj_in_obj(vertices_of_obj_in_obj),
      m_vertices_of_robot_in_robot(vertices_of_robot_in_robot),
      m_center_to_grapsing_in_robot(center_to_grapsing_in_robot),
      m_grasping_points_in_obj(grasping_points_in_obj),
      m_rnum(rnames.size()),
      m_rnames(rnames),
      m_actual_configuration(3+2*rnames.size())
  {
    m_base_angles.resize(m_rnames.size());
    std::fill(m_base_angles.begin(), m_base_angles.end(), 0);
  }
  void setBaseOrientation(const std::vector<double> angles);
  void computePoses(const std::string& name, const Eigen::VectorXd& config);
  Eigen::Isometry3d getPose(const std::string& name);
  Eigen::Vector3d getPointOnObject(Eigen::Vector3d& point_in_center_frame);
  Eigen::Vector3d getPointOnBase(const std::string& name, const Eigen::Vector3d& point_in_base_frame);
};

#endif // FORMATION_H
