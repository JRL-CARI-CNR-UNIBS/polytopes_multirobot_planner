#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <random_numbers/random_numbers.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <graph_core/sampler.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/metrics.h>

#include <cvx_motion_planning/solvers/cvx_solver.h>

#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <iris/iris.h>
#include <iris/geometry.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
//  ros::NodeHandle nh("convex_motion_planning");
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_solver");

//  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
//  ros::console::notifyLoggerLevelsChanged();
//  ros::AsyncSpinner spinner(1);
//  spinner.start();
  rclcpp::Logger LOGGER = node->get_logger();

  random_numbers::RandomNumberGenerator rng;
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//  moveit::planning_interface::MoveGroupInterface move_group_ur1("ur1");
  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);


  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcast = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  std::unique_ptr<tf2_ros::Buffer> tfBuffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfStaticBroadcast = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);

  //ros::Publisher poly_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("polyhedrons",10);
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub = node->create_publisher<geometry_msgs::msg::PolygonStamped>("polyhedrons", 10);
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(LOGGER, "Planning scene: waiting for subscribers");
  }

  // Spawn Objects
  // ==========================
  RCLCPP_INFO(LOGGER,"Start Object creation");

  moveit_msgs::msg::CollisionObject coll_obj;
  coll_obj.header.frame_id = "world";
  coll_obj.header.stamp = rclcpp::Clock().now();
  coll_obj.operation = coll_obj.ADD;
  const int n_objects = 6;
  double box_pos[n_objects][2] = {{1, 1}, {1, 2}, {1, 3}, {2, 1}, {3, 1}, {4, 1}};
  shape_msgs::msg::SolidPrimitive solid;
  solid.type = solid.BOX;
  solid.dimensions.resize(3);
  solid.dimensions[0] = 0.4;
  solid.dimensions[1] = 0.4;
  solid.dimensions[2] = 0.1;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  for(int idx = 0; idx < n_objects; idx++)
  {
    coll_obj.id = "box" + std::to_string(idx);
    box_pose.position.x = box_pos[idx][0];
    box_pose.position.y = box_pos[idx][1];
    box_pose.position.z = 0;
    coll_obj.primitives.clear();
    coll_obj.primitive_poses.clear();
    coll_obj.primitives.push_back(solid);
    coll_obj.primitive_poses.push_back(box_pose);
    collision_objects.push_back(coll_obj);
  }
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.collision_objects = collision_objects;
  planning_scene_diff_publisher->publish(planning_scene_msg);
  // ==========================

  RCLCPP_INFO(LOGGER,"Start solver configuration");
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, "all");
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::InformedSamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(Eigen::Vector2d(-7,-7), Eigen::Vector2d(7,7), Eigen::Vector2d(-7,-7), Eigen::Vector2d(7,7));
  RCLCPP_INFO(LOGGER,"Solver creation");
  pathplan::CVXSolverPtr solver = std::make_shared<pathplan::CVXSolver>(metrics, checker, sampler, 2);

  solver->config(node);
  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(Eigen::Vector3d(0, 0, rng.uniform01()*2*M_PI));
  RCLCPP_INFO(LOGGER,"Solver->addStart");
  solver->addStart(start_node);
  pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(Eigen::Vector3d(5, 5, rng.uniform01()*2*M_PI));
  RCLCPP_INFO(LOGGER,"Solver->addGoal");
  solver->addGoal(goal_node);
  pathplan::PathPtr solution;
  RCLCPP_INFO(LOGGER,"Start solver");
  while(!solver->update(solution));
  RCLCPP_INFO(LOGGER,"End solver");

  std::vector<pathplan::NodePtr> node_list = solution->getNodes();
  std::map<pathplan::NodePtr, std::string> tfnode_names;
  int idx = 0;
  for(pathplan::NodePtr node : node_list)
  {
    tfnode_names[node] = "node_" + std::to_string(idx++);
    geometry_msgs::msg::TransformStamped tfpoint;
    tfpoint.child_frame_id = tfnode_names[node];
    tfpoint.header.frame_id = "world";
    tfpoint.header.stamp = rclcpp::Clock().now();
    tfpoint.transform.translation.x = node->getConfiguration()(0);
    tfpoint.transform.translation.y = node->getConfiguration()(1);
    Eigen::Quaterniond new_rotation; new_rotation = Eigen::AngleAxisd(node->getConfiguration()(2), Eigen::Vector3d::UnitZ());
    RCLCPP_DEBUG_STREAM(LOGGER,"main() -> new_rotation: " << new_rotation.w() << " " << new_rotation.x() << "i " << new_rotation.y() << "j " << new_rotation.z() << "k");
    tfpoint.transform.rotation = Eigen::toMsg(new_rotation);
    tfStaticBroadcast->sendTransform(tfpoint);
    RCLCPP_WARN(LOGGER,"published %s", tfnode_names[node].c_str());
  }

  RCLCPP_INFO(LOGGER, "Cartesia Tree built");

  std::vector<pathplan::ConnectionPtr> connection_list = solution->getConnections();
  std::multimap<pathplan::ConnectionPtr, PolyhedronContainerPtr> map_connection_to_poly;
  geometry_msgs::msg::Point point; geometry_msgs::msg::Point32 point32; Eigen::Vector3d p3d;
  for(pathplan::ConnectionPtr& conn : connection_list)
  {
    auto mm_poly = map_connection_to_poly.equal_range(conn);
//    jsk_recognition_msgs::PolygonArray poly_array_msg;
    std::vector<geometry_msgs::msg::PolygonStamped> poly_msg_array;
    for(auto polyc = mm_poly.first; polyc != mm_poly.second; ++polyc)
    {
      std::vector<Eigen::VectorXd> vecp = polyc->second->getPolyhedron().generatorPoints(); // Sperando li restituisca in ordine
      geometry_msgs::msg::PolygonStamped poly_msg;
//      poly_msg.polygon.points.resize(vecp.size());
      poly_msg.header.frame_id = "world";
      for(Eigen::VectorXd p : vecp)
      {
        assert(p.rows() == 2);
        p3d << p, 0.0;
        point = Eigen::toMsg(p3d);
        point32.x = point.x;
        point32.y = point.y;
        point32.z = point.z;
        poly_msg.polygon.points.push_back(point32);
      }
      poly_msg_array.push_back(poly_msg);
    }
    for(geometry_msgs::msg::PolygonStamped pp : poly_msg_array)
      poly_pub->publish(pp);
  }

  RCLCPP_DEBUG(LOGGER,"TF published");

  while(true)
  {
    for(pathplan::NodePtr node : node_list)
    {
      geometry_msgs::msg::TransformStamped world_to_main_object = tfBuffer->lookupTransform("center_formation","world",tf2::TimePointZero);
      world_to_main_object.header.frame_id = "world";
      world_to_main_object.child_frame_id = "center_formation";
      world_to_main_object.header.stamp = rclcpp::Clock().now();
      world_to_main_object.transform.translation.x = node->getConfiguration()(0);
      world_to_main_object.transform.translation.y = node->getConfiguration()(1);
      Eigen::AngleAxisd aa(node->getConfiguration()(2), Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond qq(aa);
      world_to_main_object.transform.rotation.w = qq.w();
      world_to_main_object.transform.rotation.z = qq.z();
      tfBroadcast->sendTransform(world_to_main_object);
      rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(2));
    }
  }

  return 0;
}
