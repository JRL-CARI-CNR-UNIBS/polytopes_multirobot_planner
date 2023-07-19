#include "polytopes_centralized_planner/formation.h"

void Formation::setBaseOrientation(const std::vector<double> angles)
{
  m_base_angles = angles;
}

void Formation::computePoses(const std::string& name, const Eigen::VectorXd& config)
{
  Eigen::Vector3d t({config(0), config(1), 0});
  Eigen::AngleAxisd Rt(config(2), Eigen::Vector3d::UnitZ());
  std::vector<Eigen::Vector3d> robot_to_grasp_in_robot(m_rnum);
  for (size_t idx = 0; idx < robot_to_grasp_in_robot.size(); ++idx){
     robot_to_grasp_in_robot[idx] = (Eigen::AngleAxisd(m_base_angles[idx],
                                                       Eigen::Vector3d::UnitZ())
                                     * Eigen::Vector3d::UnitX()
                                     );
     robot_to_grasp_in_robot[idx] *= -config(3 + idx);
  }

  for(size_t idx = 0; idx < m_rnum; ++idx)
  {
    Eigen::AngleAxisd rotation_on_grasp( config(3+m_rnum+idx), Eigen::Vector3d::UnitZ() );
    m_robot_poses[m_rnames[idx]].translation() = t + Rt *
        (
          m_grasping_points_in_obj.col(idx)
          + rotation_on_grasp
            * (-robot_to_grasp_in_robot[idx])
        );
    m_robot_poses[m_rnames[idx]].linear() = (Rt
        * Eigen::AngleAxisd(std::atan2(m_grasping_points_in_obj.col(idx)(0),
                                       m_grasping_points_in_obj.col(idx)(1)),
                            Eigen::Vector3d::UnitZ())
        * rotation_on_grasp
        * Eigen::AngleAxisd(m_base_angles[idx], Eigen::Vector3d::UnitZ())
        ).toRotationMatrix();
  }
}

Eigen::Isometry3d Formation::getPose(const std::string& name)
{
  if(!is_configured)
  {
    return m_robot_poses[name];
  }
  else
  {
    std::cerr << "Cannot get poses. Poses not computed. Returning identity isometry" << std::endl;
    return Eigen::Isometry3d::Identity();
  }
}

Eigen::Vector3d Formation::getPointOnBase(const std::string& name, const Eigen::Vector3d& point_in_base_frame)
{
  if(!is_configured)
    return m_robot_poses[name] * point_in_base_frame;
  else
  {
    std::cerr << "Cannot get point position. Poses not computed. Returning zero vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }
}

Eigen::Vector3d Formation::getPointOnObject(Eigen::Vector3d& point_in_center_frame)
{
  if(!is_configured)
    return Eigen::Vector3d({m_actual_configuration(0),m_actual_configuration(1),0})
        + Eigen::AngleAxisd(m_actual_configuration(2), Eigen::Vector3d::UnitZ())
          * point_in_center_frame;
  else
  {
    std::cerr << "Cannot get point position. Poses not computed. Returning zero vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }
}
