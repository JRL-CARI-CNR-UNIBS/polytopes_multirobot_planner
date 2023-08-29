//#include <polytopes_centralized_planner/nodes/centralized_planner_node.hpp>

#include <cmath>
#include <algorithm>
#include <functional>
#include <future>

#include <polytopes_centralized_planner/solvers/polytopes_solver.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <graph_core/metrics.h>
#include <graph_core/collision_checker.h>
#include <graph_core/sampler.h>

#include <nav_msgs/srv/get_plan.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

class CentralizedPlanner : public rclcpp::Node//rclcpp_lifecycle::LifecycleNode
{
protected:
  pathplan::MetricsPtr metrics;
  pathplan::CollisionCheckerPtr cchecker;
  pathplan::SamplerPtr sampler;
  pathplan::CVXSolverPtr solver;

  tf2_ros::Buffer::SharedPtr tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using NavPosesHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr planning_service_;

  std::vector<rclcpp_action::Client<NavigateThroughPoses>::SharedPtr> nav_poses_clnt_vector;
//  std::vector<geometry_msgs::msg::PoseStamped> goal_publishers_;

protected:
  const std::string world_frame_param_ {"world_frame"};
  const std::string default_world_frame_ {"/map"};
  std::string world_frame_;

public:
  CentralizedPlanner(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
    : Node("centralized_planner_node", opt),
      metrics(std::make_shared<pathplan::Metrics>()),
      cchecker(std::make_shared<pathplan::CollisionChecker>())
  {
    using namespace std::placeholders;
    solver = std::make_shared<pathplan::CVXSolver>(metrics, cchecker, sampler);
    planning_service_ =
        this->create_service<nav_msgs::srv::GetPlan>
        ("centralized_planner",
         std::bind(&CentralizedPlanner::plan, this, _1, _2));
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // Parameters
    this->declare_parameter("allow_undeclared_parameters",true);
  }

  // TODO: Change srv
  void plan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
            std::shared_ptr<nav_msgs::srv::GetPlan::Response> res)
  {
    this->get_parameter_or(world_frame_param_, world_frame_, default_world_frame_);
    if(req->goal.header.frame_id != world_frame_)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "goal is not refered to the world frame specified.");
      return;
    }
    solver->config(this->shared_from_this());
    std::vector<std::string> bases = solver->getRobotBases();
    std::vector<Eigen::Isometry3d> eigen_tf_from_map_to_bases;
    eigen_tf_from_map_to_bases.resize(bases.size());
    auto now = this->get_clock()->now();
    for(size_t idx = 0; idx < bases.size(); idx++)
    {
      try{
        eigen_tf_from_map_to_bases[idx] = tf2::transformToEigen(
              tfBuffer_->lookupTransform(bases[idx], world_frame_, now)
              );
      }
      catch(const tf2::TransformException & ex)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot transform from " << world_frame_
                                                << " to " << bases[idx] << ". Interrupting");
        return;
      }
    }

//    Eigen::Vector3d formation_center;
//    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
//    formation_center = std::accumulate(eigen_tf_from_map_to_bases.begin(),
//                                       eigen_tf_from_map_to_bases.end(),
//                                       zero,
//                                       [](const Eigen::Vector3d& a, const Eigen::Isometry3d& b){
//      return a + b.translation().matrix();
//    });
//    solver->addStart(std::make_shared<pathplan::Node>(formation_center/bases.size()));

    /*
     * get_poses_from_amcl
     * get_initial_config_from_somewhere
     * compute_poses
     */
    Eigen::Isometry3d formation_center;
    std::vector<Eigen::Isometry3d> base_poses;
    std::transform(solver->getRobotBases().begin(),
                   solver->getRobotBases().end(),
                   base_poses.begin(),
                   [this](const std::string& s){
      if(tfBuffer_->canTransform(s, world_frame_, this->get_clock()->now()))
      {
        return tf2::transformToEigen(
              tfBuffer_->lookupTransform(s, world_frame_, this->get_clock()->now())
              );
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Transformation from %s to %s not found! Shutting down...",
                     world_frame_.c_str(),
                     s.c_str());
        rclcpp::shutdown();
        return Eigen::Isometry3d::Identity();
      }
    });

    Eigen::Isometry3d goal_pose;
    tf2::fromMsg(req->goal.pose, goal_pose);
    Eigen::Vector3d goal_sample({goal_pose.translation()(0),
                                goal_pose.translation()(1),
                                Eigen::AngleAxisd().fromRotationMatrix(goal_pose.linear()).angle()});

    solver->addGoal(goal_sample);

    pathplan::PathPtr result_path;
    while(solver->update(result_path)); // TODO: check risultato
    std::vector<Eigen::VectorXd> result_path_waypoints = result_path->getWaypoints();
    std::map<std::string, std::vector<Eigen::Isometry3d>>
        path_of_configurations = solver->getConfigFromPath(result_path);

    std::vector<std::string> robot_names = solver->getRobots();
    nav_poses_clnt_vector.resize(robot_names.size());
    std::transform(robot_names.begin(),
                   robot_names.end(),
                   nav_poses_clnt_vector.begin(),
                   [this](const std::string& rname){
      return rclcpp_action::create_client<NavigateThroughPoses>(this, rname + "_action");
    });

    std::vector<rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions>
        send_goal_options_vector;
    send_goal_options_vector.resize(nav_poses_clnt_vector.size());

    for(auto& option : send_goal_options_vector)
    {
      option.goal_response_callback =
        std::bind(&CentralizedPlanner::goal_response_callback, this, std::placeholders::_1);
      option.feedback_callback =
        std::bind(&CentralizedPlanner::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      option.result_callback =
        std::bind(&CentralizedPlanner::result_callback, this, std::placeholders::_1);
    }

    // for each robot
    for(size_t idx=0; idx < nav_poses_clnt_vector.size(); ++idx)
    {
      NavigateThroughPoses::Goal goal;
//      for(size_t jdx = 0; jdx < solver->getSolution()->getWaypoints().size(); ++jdx)
//      {
//        goal.poses[jdx].header.frame_id = "map";
//        goal.poses[jdx].pose.orientation.z = std::sin(result_path_waypoints[jdx](2)*0.5);
//        goal.poses[jdx].pose.orientation.w = std::cos(result_path_waypoints[jdx](2)*0.5);
//        goal.poses[jdx].pose = tf2::toMsg(path_of_configurations[robot_names[idx]][jdx]);
//      }
      for(size_t jdx = 0; jdx < solver->getSolution()->getWaypoints().size(); ++jdx)
      {
        goal.poses[jdx].header.frame_id = world_frame_;
        goal.poses[jdx].pose = tf2::toMsg(path_of_configurations[robot_names[idx]][jdx]);
      }
      RCLCPP_INFO(this->get_logger(), "Calling action: NavigateThroughPoses on %s", robot_names[idx].c_str());
      this->nav_poses_clnt_vector[idx]->async_send_goal(goal, send_goal_options_vector[idx]);
    }
  }

  void goal_response_callback(const NavPosesHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      // TODO: Interrompi anche gli altri robot: o tutti, o nessuno
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(NavPosesHandle::SharedPtr,
                         const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
  {

  }

  void result_callback(const NavPosesHandle::WrappedResult& result)
  {
    // TODO: Interrompi anche gli altri robot: o tutti, o nessuno
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_ERROR(this->get_logger(), "Navigation Completed");
  }

};

//RCLCPP_COMPONENTS_REGISTER_NODE(CentralizedPlanner);

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.allow_undeclared_parameters(true);
  rclcpp::spin(std::make_shared<CentralizedPlanner>(opt));
  rclcpp::shutdown();
  return 0;
}
