#include "polytopes_centralized_planner/formation.h"

void Formation::setBaseOrientation(const std::vector<double> angles)
{
  m_base_angles = angles;
}

Eigen::Isometry3d Formation::computePose(const std::string& name, const Eigen::VectorXd& config) const
{
  Eigen::Vector3d t({config(0), config(1), 0});
  Eigen::AngleAxisd Rt(config(2), Eigen::Vector3d::UnitZ());
  unsigned int idx = std::distance(m_rnames.begin(), std::find(m_rnames.begin(), m_rnames.end(), name));
  std::vector<Eigen::Vector3d> robot_to_grasp_in_robot(m_rnum);
  robot_to_grasp_in_robot[idx] = (Eigen::AngleAxisd(m_base_angles[idx],
                                                    Eigen::Vector3d::UnitZ())
                                  * Eigen::Vector3d::UnitX()
                                  );
  robot_to_grasp_in_robot[idx] *= -config(3 + idx);
  Eigen::AngleAxisd rotation_on_grasp( config(3+m_rnum+idx), Eigen::Vector3d::UnitZ() );
  Eigen::Isometry3d robot_pose;
  robot_pose.translation() = t + Rt *
      (
        m_grasping_points_in_obj.col(idx)
        + rotation_on_grasp
          * (-robot_to_grasp_in_robot[idx])
      );
  robot_pose.linear() = (Rt
      * Eigen::AngleAxisd(std::atan2(m_grasping_points_in_obj.col(idx)(0),
                                     m_grasping_points_in_obj.col(idx)(1)),
                          Eigen::Vector3d::UnitZ())
      * rotation_on_grasp
      * Eigen::AngleAxisd(m_base_angles[idx], Eigen::Vector3d::UnitZ())
      ).toRotationMatrix();

  return robot_pose;
}

Eigen::Isometry3d Formation::getPose(const std::string& name, const Eigen::VectorXd& config) const
{
  return computePose(name, config);
}

std::vector<Eigen::Vector3d> Formation::getPointsOnBase(const std::string& name, const Eigen::VectorXd& config, const std::vector<Eigen::Vector3d>& point_in_base_frame) const
{
    std::vector<Eigen::Vector3d> points_in_map(point_in_base_frame.size());
    Eigen::Isometry3d robot_frame_in_map = computePose(name, config);
    std::transform(point_in_base_frame.begin(), point_in_base_frame.end(), points_in_map.begin(),
                   [robot_frame_in_map](const Eigen::Vector3d& v){
      return  robot_frame_in_map * v;
    });
    return points_in_map;
}

std::vector<Eigen::Vector3d> Formation::getPointsOnBase(const std::string& name, const Eigen::VectorXd& config, const Eigen::Matrix3Xd& point_in_base_frame) const
{
  std::vector<Eigen::Vector3d> p;
  p.resize(point_in_base_frame.cols());
  for(size_t idx = 0; idx < p.size(); ++idx)
  {
    p[idx] = point_in_base_frame.col(idx);
  }
  return getPointsOnBase(name, config, p);
}

std::vector<Eigen::Vector3d> Formation::getPointsOnObject(const Eigen::VectorXd& config, const std::vector<Eigen::Vector3d>& point_in_center_frame) const
{

  std::vector<Eigen::Vector3d> points_in_map(point_in_center_frame.size());
  std::transform(point_in_center_frame.begin(), point_in_center_frame.end(), points_in_map.begin(),
                 [config](const Eigen::Vector3d& v){
    return Eigen::Vector3d({config(0),config(1),0})
        + Eigen::AngleAxisd(config(2), Eigen::Vector3d::UnitZ()) * v;
  });
  return points_in_map;
}

std::vector<Eigen::Vector3d> Formation::getPointsOnObject(const Eigen::VectorXd& config, const Eigen::Matrix3Xd& point_in_center_frame) const
{
  std::vector<Eigen::Vector3d> p;
  p.resize(point_in_center_frame.cols());
  for(size_t idx = 0; idx < p.size(); ++idx)
  {
    p[idx] = point_in_center_frame.col(idx);
  }
  return getPointsOnObject(config, p);
}
