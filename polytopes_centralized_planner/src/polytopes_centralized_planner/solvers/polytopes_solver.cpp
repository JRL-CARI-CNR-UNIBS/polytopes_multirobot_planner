#include "polytopes_centralized_planner/solvers/polytopes_solver.hpp"

geometry_msgs::msg::Point p32_to_p(const geometry_msgs::msg::Point32& p32)
{
  geometry_msgs::msg::Point p;
  p.x = p32.x;
  p.y = p32.y;
  p.z = p32.z;
  return p;
}

namespace pathplan
{
bool CVXSolver::setProblem(const double &max_time)
{
  init_ = false;
  if (!start_tree_)
    return false;
  if (!goal_node_)
    return false;
  goal_cost_ = goal_cost_fcn_->cost(goal_node_);


  fproblem_.newProblem();
  fproblem_.setBounds(configuration_bounds_);
  fproblem_.setObjVertices(object_vertices_in_obj_, grasping_vertices_in_obj_);
  fproblem_.setRobotsVertices(robot_vertices_, grasping_vertices_in_robot_);
  fproblem_.setObstacles(obstacles_);

  init_ = true;
  // TODO: Direct connection

  path_cost_ = std::numeric_limits<double>::infinity();
  cost_ = path_cost_ + goal_cost_;
  return true;
}

bool CVXSolver::config(const std::shared_ptr<rclcpp::Node>& node)
{
  node_ = node;
  RCLCPP_DEBUG(node->get_logger(), "CVXSolver::config()");

  tf2_ros::Buffer::SharedPtr tfBuffer= std::make_shared<tf2_ros::Buffer>(node->get_clock()); // temporary clock
  std::shared_ptr<tf2_ros::TransformListener> tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  std::vector<std::string> robot_to_load;
  std::string param_prefix = "polytope_motion_planning.";
  std::string robot_param_prefix = param_prefix + "robots.";
  std::string obj_param_prefix = param_prefix + "main_object.";

  if(!node->get_parameter(robot_param_prefix + "robot_to_load", robot_to_load))
  {
    RCLCPP_FATAL(node->get_logger(), "%srobot_to_load not defined.", (node->get_namespace()+robot_param_prefix).c_str());
    throw std::runtime_error("");
  }
  std::vector<std::string> base_names;
  if(!node->get_parameter(robot_param_prefix + "base_names", base_names))
  {
    RCLCPP_FATAL(node->get_logger(), "%sbase_names not defined.", (node->get_namespace()+robot_param_prefix).c_str());
    throw std::runtime_error("");
  }
  robot_loaded_ = std::move(robot_to_load);
  base_names_ = std::move(base_names);

  // Utility variables
  std::vector<double> tmp_2d_vector;
  std::vector<double> tmp_3d_vector;
  int n_points;


  // Bounds for optimization variable
  Eigen::MatrixXd configuration_bounds(2, 3 + 2*robot_to_load.size());
  if(!node->get_parameter(param_prefix + "bounds.pose.x",tmp_2d_vector))
  {
    RCLCPP_FATAL(node->get_logger(), "%s.bounds.pose.x not defined.", (node->get_namespace()+robot_param_prefix).c_str());
    throw std::runtime_error("");
  }
  configuration_bounds(0,0) = tmp_2d_vector.at(0);
  configuration_bounds(1,0) = tmp_2d_vector.at(1);
  if(!node->get_parameter(param_prefix + "bounds.pose.y",tmp_2d_vector))
  {
    RCLCPP_FATAL(node->get_logger(), "%s.bounds.pose.x not defined.", (node->get_namespace()+robot_param_prefix).c_str());
    throw std::runtime_error("");
  }
  configuration_bounds(0,1) = tmp_2d_vector.at(0);
  configuration_bounds(1,1) = tmp_2d_vector.at(1);
  if(!node->get_parameter(param_prefix + "bounds.pose.theta",tmp_2d_vector))
  {
    RCLCPP_FATAL(node->get_logger(), "%s.bounds.pose.theta not defined.", (node->get_namespace()+robot_param_prefix).c_str());
    throw std::runtime_error("");
  }
  configuration_bounds(0,2) = tmp_2d_vector.at(0);
  configuration_bounds(1,2) = tmp_2d_vector.at(1);

  for(int idx = 0; idx < robot_to_load.size(); idx++)
  {
    if(!node->get_parameter(param_prefix + "bounds.pose.distances." + robot_to_load[idx], tmp_2d_vector))
    {
      RCLCPP_FATAL(node->get_logger(), "%s.bounds.pose.distances.%s not defined.", (node->get_namespace()+robot_param_prefix).c_str(), robot_to_load[idx].c_str());
      throw std::runtime_error("");
    }
    configuration_bounds(0,3 + idx) = tmp_2d_vector.at(0);
    configuration_bounds(1,3 + idx) = tmp_2d_vector.at(1);
    if(!node->get_parameter(param_prefix + "bounds.pose.grasp_angle." + robot_to_load[idx], tmp_2d_vector))
    {
      RCLCPP_FATAL(node->get_logger(), "%sbounds.pose.grasp_angle.%s not defined.", (node->get_namespace()+robot_param_prefix).c_str(), robot_to_load[idx].c_str());
      throw std::runtime_error("");
    }
    configuration_bounds(0,3 + robot_to_load.size() + idx) = tmp_2d_vector.at(0);
    configuration_bounds(1,3 + robot_to_load.size() + idx) = tmp_2d_vector.at(1);
  }
  configuration_bounds_ = configuration_bounds;


  // Workspace
  Eigen::MatrixXd workspace_vertices;
  if(!node->get_parameter(param_prefix+"workspace_vertices.n_points",n_points))
  {
    RCLCPP_FATAL(node->get_logger(), "%sworkspace_vertices.n_points is not set.", (node->get_namespace()+param_prefix).c_str());
    throw std::runtime_error("");
  }
  workspace_vertices.resize(2,n_points);
  for(int idx = 0; idx < n_points; idx++)
  {
    if(!node->get_parameter(param_prefix+"workspace_vertices.p"+std::to_string(idx+1),tmp_2d_vector))
    {
      RCLCPP_FATAL(node->get_logger(), "%sworkspace_vertices.p%d is not set.", (node->get_namespace()+param_prefix).c_str(), idx+1);
      throw std::runtime_error("");
    }
    workspace_vertices(0,idx) = tmp_2d_vector.at(0);
    workspace_vertices(1,idx) = tmp_2d_vector.at(1);
  }

  sampler_ = std::make_shared<InformedSampler>(workspace_vertices.rowwise().minCoeff(),
                                               workspace_vertices.rowwise().maxCoeff(),
                                               workspace_vertices.rowwise().minCoeff(),
                                               workspace_vertices.rowwise().maxCoeff());

  Eigen::MatrixXd line(2, workspace_vertices.cols());
  for(int idx = 0; idx < workspace_vertices.cols(); idx++)
  {
    line << workspace_vertices.col(idx), workspace_vertices.col((idx+1) % workspace_vertices.cols());
    workspace_bounds_.push_back(line);
  }

  // Transported object
  std::string main_object_frame;
  if(!node->get_parameter(obj_param_prefix + "frame", main_object_frame))
  {
    RCLCPP_FATAL(node->get_logger(), "%sframe is missing.", (node->get_namespace() + obj_param_prefix).c_str());
    throw std::runtime_error("");
  }
  if(!node->get_parameter(obj_param_prefix+"vertices.n_points", n_points))
  {
    RCLCPP_ERROR(node->get_logger(), "%svertices.n_points is not set", (node->get_namespace() + robot_param_prefix).c_str());
    throw std::runtime_error("");
  }
  for(int idx=0; idx < n_points; idx++)
  {
    if(!node->get_parameter(robot_param_prefix+"vertices.p"+std::to_string(idx+1), tmp_3d_vector))
    {
      RCLCPP_ERROR(node->get_logger(), "%svertices.p%d is not set", (node->get_namespace() + robot_param_prefix).c_str(), idx+1);
      throw std::runtime_error("");
    }
    object_vertices_in_obj_(0,idx) = tmp_3d_vector.at(0);
    object_vertices_in_obj_(1,idx) = tmp_3d_vector.at(1);
    object_vertices_in_obj_(2,idx) = tmp_3d_vector.at(2);
  }


  // Grasping points wrt obj and base_footprints
  std::vector<std::string> grasping_frames;
  if(!node->get_parameter(obj_param_prefix + "grasping_frame", grasping_frames))
  {
    RCLCPP_FATAL(node->get_logger(), "%sgrasping_frame is missing.", (node->get_namespace() + obj_param_prefix).c_str());
    throw std::runtime_error("");
  }

  grasping_vertices_in_obj_.resize(3,grasping_frames.size());
  grasping_vertices_in_robot_.resize(3,grasping_frames.size());
  for(int idx = 0; idx < grasping_frames.size(); idx++)
  {
    if(!tfBuffer->canTransform(grasping_frames.at(idx), main_object_frame, tf2::TimePointZero, tf2::Duration(1000L)))
    {
      RCLCPP_ERROR(node->get_logger(), "Cannot transform from %s to %s",main_object_frame.c_str(), grasping_frames.at(idx).c_str());
      throw std::runtime_error("");
    }
    geometry_msgs::msg::TransformStamped tf_obj_to_grasp = tfBuffer->lookupTransform(grasping_frames.at(idx), main_object_frame, tf2::TimePointZero);
    grasping_vertices_in_obj_.col(idx) = tf2::transformToEigen(tf_obj_to_grasp).matrix().block<3,1>(0,3);
  }
  for(int idx = 0; idx < grasping_frames.size(); idx++)
  {
    if(!tfBuffer->canTransform(grasping_frames.at(idx), base_names.at(idx), tf2::TimePointZero, tf2::Duration(1000L)))
    {
      RCLCPP_ERROR(node->get_logger(), "Cannot transform from %s to %s", base_names.at(idx).c_str(), grasping_frames.at(idx).c_str());
      throw std::runtime_error("");
    }
    geometry_msgs::msg::TransformStamped T_base_robot_to_grasp = tfBuffer->lookupTransform(grasping_frames.at(idx), base_names.at(idx), tf2::TimePointZero);
    grasping_vertices_in_robot_.col(idx) = tf2::transformToEigen(T_base_robot_to_grasp).matrix().block<3,1>(0,3);
  }

  // Base verticies
  Eigen::MatrixXd robot_vertices;
  for (std::string& robot_name : robot_to_load)
  {
    if(!node->get_parameter(robot_param_prefix+"vertices."+robot_name+".n_points", n_points))
    {
      RCLCPP_ERROR(node->get_logger(), "%svertices.%s.n_points is not set", (node->get_namespace() + robot_param_prefix).c_str(), robot_name.c_str());
      throw std::runtime_error("");
    }
    for(int idx=0; idx < n_points; idx++)
    {
      if(!node->get_parameter(robot_param_prefix+"vertices."+robot_name+".p"+std::to_string(idx+1), tmp_3d_vector))
      {
        RCLCPP_ERROR(node->get_logger(), "%svertices.%s.p%d is not set", (node->get_namespace() + robot_param_prefix).c_str(), robot_name.c_str(), idx+1);
        throw std::runtime_error("");
      }
      robot_vertices(0,idx) = tmp_3d_vector.at(0);
      robot_vertices(1,idx) = tmp_3d_vector.at(1);
      robot_vertices(2,idx) = tmp_3d_vector.at(2);
    }
    robot_vertices_.push_back(robot_vertices);
  }

//  includeObstacleFromScene();
  std::string simple_map_service;
  node->get_parameter_or<std::string>("simplified_map_topic", simple_map_service, "/simplified_map");
  auto simplified_map_clnt = node->create_client<polytopes_centralized_planner_msgs::srv::MapWithBoundingBoxes>(simple_map_service);

  auto simplified_map_response = simplified_map_clnt->async_send_request(std::make_shared<polytopes_centralized_planner_msgs::srv::MapWithBoundingBoxes::Request>());
  if(rclcpp::spin_until_future_complete(node, simplified_map_response) == rclcpp::FutureReturnCode::SUCCESS)
  {
    for(std::size_t jdx = 0; jdx < simplified_map_response.get()->polygon_list.size(); jdx++)
    {
      std::string obstacle_name {"obstacle_" + std::to_string(jdx)};
      std::vector<Eigen::Matrix2d> lines;
      lines.resize(simplified_map_response.get()->polygon_list.at(jdx).polygon.points.size());
      for(std::size_t idx = 0; idx < lines.size(); idx++)
      {
        Eigen::Vector3d v1, v2;
        tf2::fromMsg(p32_to_p(simplified_map_response.get()->polygon_list.at(jdx).polygon.points.at(idx)),
                     v1);
        tf2::fromMsg(p32_to_p(simplified_map_response.get()->polygon_list.at(jdx).polygon.points.at(idx+1 % lines.size())),
                     v2);
        lines.at(idx) << v1.head<2>(), v2.head<2>();
      }
      obstacles_[obstacle_name] = lines;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "cannot call service: " << simple_map_service);
    std::runtime_error("Cannot call service. Exiting...");
  }

  bool ret = TreeSolver::config(node);
  RCLCPP_DEBUG(node->get_logger(), "Informed Sampling forced to: FALSE");
  informed_ = false;
  RCLCPP_DEBUG(node->get_logger(), "~CVXSolver::config()");

  return ret;

}

// Bounding box
//bool CVXSolver::getConvexVerticesFromShape(const shapes::ShapeConstPtr& shape, const Eigen::Isometry3d& pose, Eigen::MatrixXd& vertices)
//{
//  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::getConvexVerticesFromShape()");
//  bodies::AABB aabb;
//  bodies::Body* body = bodies::createBodyFromShape(shape->clone()); // <--- clone? boh
////  RCLCPP_DEBUG_STREAM(node_->get_logger(), "CVXSolver::getConvexVerticesFromShape() -> pose: " << pose.matrix());
//  body->setPose(pose);
//  body->computeBoundingBox(aabb);
//  Eigen::Vector2d tmp;
//  for (unsigned int idx = 0; idx < 4; idx++)
//  {
////    RCLCPP_DEBUG_STREAM(node_->get_logger(), "CVXSolver::getConvexVerticesFromShape() -> corner: " << aabb.corner(static_cast<Eigen::AlignedBox<double, 3>::CornerType>(idx)));
//    vertices.col(idx) = aabb.corner(static_cast<Eigen::AlignedBox<double, 3>::CornerType>(idx)).head(2);
//  }
//  // Switch TopLeft with TopRight to obtain a contour
//  tmp = vertices.col(2);
//  vertices.col(2) = vertices.col(3);
//  vertices.col(3) = tmp;
//  return true;
//}

bool CVXSolver::addStart(const NodePtr& start_node, const double &max_time)
{
  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::addStart()");
  if (!configured_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Solver is not configured.");
    return false;
  }
  solved_ = false;
  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::addStart() -> create tree");
  start_tree_ = std::make_shared<Tree>(start_node, max_distance_, checker_, metrics_, use_kdtree_);

  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::AddStart() -> Polyhedron computation");
  PolyhedronContainerPtr poly;
  if(!getPolyhedronFromNode(start_node, poly))
  {
    RCLCPP_ERROR(node_->get_logger(), "Start point: no polyhedron found");
    return false;
  }
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "CVXSolver::AddStart() -> Polyhedron: " << poly->getPolyhedron().getA());
  poly_list_.push_back(poly);

  //

  setProblem(max_time);

  RCLCPP_DEBUG(node_->get_logger(), "~CVXSolver::addStart()");
  return true;
}

bool CVXSolver::addStartTree(const TreePtr& start_tree, const double &max_time)
{
  RCLCPP_FATAL(node_->get_logger(), "NOT IMPLEMENTED: CVXSolver::addStartTree");
  return false;
}

bool CVXSolver::addGoal(const NodePtr &goal_node, const double &max_time)
{
  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::addGoal");
  if(!configured_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Solver is not configured.");
    return false;
  }
  solved_ = false;
  goal_node_ = goal_node;

  goal_cost_=goal_cost_fcn_->cost(goal_node);
  if(!getPolyhedronFromNode(goal_node, goal_poly_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal point: no polyhderon found");
    return false;
  }

  setProblem(max_time);

  RCLCPP_DEBUG(node_->get_logger(), "~CVXSolver::addGoal");
  return true;
}

bool CVXSolver::addGoal(const Eigen::Vector3d& sample)
{
  Eigen::VectorXd goal;
  if(configurationComputeOnly(sample, goal))
  {
    NodePtr goal_node = std::make_shared<Node>(goal);
    addGoal(goal_node);
    return true;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal configuration could not be computed");
    return false;
  }
}

bool CVXSolver::update(PathPtr& solution)
{
  PATH_COMMENT("CVXSolver::update() with sampling");
  if(solved_)
  {
    solution = solution_;
    return true;
  }

  if (sampler_->collapse())
    return false;

  Eigen::VectorXd sample;
  bool sample_ok = true;
  // NOTE: To keep as long as the sampler outside polyhedron is ready
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "CVXSolver::update() with sampling -> poly list dimension: " << poly_list_.size());
  do
  {    
     sample = sampler_->sample();
     sample_ok = true;
//     RCLCPP_DEBUG_STREAM(node_->get_logger(), "CVXSolver::update() with sampling -> sample: " << static_cast<Eigen::RowVectorXd>(sample));
     for(PolyhedronContainerPtr& p : poly_list_)
     {
       sample_ok = sample_ok && p->getPolyhedron().contains(sample, 1E-3); // TODO: put tollerance as parameter
       if(!sample_ok) break;
     }
  }while(!sample_ok);
  // ----

  return CVXSolver::update(sample, solution);
}

bool CVXSolver::update(const Eigen::VectorXd& sample, PathPtr& solution)
{
  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::update()");

  if(solved_)
  {
    solution = solution_;
    return true;
  }
  if(sample.rows() < sampler_->getDimension())
  {
    RCLCPP_ERROR(node_->get_logger(), "Dimensions of polyhedron seed is not that of the problem!");
    throw std::runtime_error("Dimensions of polyhedron seed is not that of the problem!");
  }
  PolyhedronContainerPtr sample_poly;
  if(!getPolyhedronFromPoint(sample, sample_poly))
  {
    return false;
  }
  Eigen::VectorXd formation_configuration(dof_ + robot_loaded_.size() * 2);
  NodePtr new_node;

  formation_configuration << sample, goal_node_->getConfiguration()(2), configuration_bounds_.colwise().mean().tail(configuration_bounds_.cols() - dof_);
  fproblem_.newProblem();
  fproblem_.setVariables(formation_configuration, configuration_bounds_);
  fproblem_.setPolyhedron(sample_poly->getPolyhedron());
  fproblem_.setGoal(sample);
  bool add_to_start = false;
  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::update() -> check configuration inside polyhedron...");
  if(fproblem_.solve(formation_configuration))
  {
    RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::update() -> Good configuration found");
    iris::Polyhedron intersec_poly;
    for(const PolyhedronContainerPtr& pc : poly_list_)
    {
      intersec_poly = sample_poly->getPolyhedron();
      intersec_poly.appendConstraints(pc->getPolyhedron());
      fproblem_.newProblem();
      fproblem_.setVariables(formation_configuration, configuration_bounds_);
      fproblem_.setPolyhedron(intersec_poly);
      fproblem_.setGoal(sample);
      RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::update() -> check configuration inside intersection...");
      if(fproblem_.solve(formation_configuration))
      {
        RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::update() -> Good configuration in intersection found");
        new_node = std::make_shared<Node>(formation_configuration.head<3>());
        std::vector<double> tmp_vec(formation_configuration.data(), formation_configuration.data() + formation_configuration.size());
        map_node_to_config[new_node] = tmp_vec;
        ConnectionPtr conn;
        for(NodePtr& node : pc->getNodes())
        {
          start_tree_->extendOnly(node, new_node, conn);
          conn->setCost(metrics_->cost(node->getConfiguration().head<2>(), new_node->getConfiguration().head<2>()));
          map_connection_to_poly_.insert(std::pair<ConnectionPtr, PolyhedronContainerPtr>(conn,pc));
        }
        for(NodePtr& node : sample_poly->getNodes())
        {
          start_tree_->extendOnly(node, new_node, conn);
          conn->setCost(metrics_->cost(node->getConfiguration().head<2>(), new_node->getConfiguration().head<2>()));
          map_connection_to_poly_.insert(std::pair<ConnectionPtr, PolyhedronContainerPtr>(conn,sample_poly));
        }
        pc->addNode(new_node);
        sample_poly->addNode(new_node);
        add_to_start = true;
      }
    }
    poly_list_.push_back(sample_poly);

  }

  if (add_to_start)
  {
    RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::update() -> trying to connect goal...");
    iris::Polyhedron goal_intersec_poly = sample_poly->getPolyhedron();
    goal_intersec_poly.appendConstraints(goal_poly_->getPolyhedron());
    fproblem_.newProblem();
    fproblem_.setVariables(formation_configuration, configuration_bounds_);
    fproblem_.setPolyhedron(goal_intersec_poly);
    fproblem_.setGoal(sample);
    if(fproblem_.solve(formation_configuration))
    {
      ConnectionPtr goal_conn = std::make_shared<Connection>(new_node, goal_node_);
      goal_conn->setCost(metrics_->cost(new_node->getConfiguration().head(2), goal_node_->getConfiguration().head(2)));
      goal_conn->add();
      PathPtr tmp_solution = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
      //solution_ = getReducedPath(tmp_solution);
      solution_ = tmp_solution;
      solution_->setTree(start_tree_);
      start_tree_->addNode(goal_node_);
      path_cost_ = solution_->cost();
      cost_=path_cost_+goal_cost_;
      //sampler_->setCost(path_cost_); //Not using informed sampling
      solution = solution_;
      solved_ = true;
      return true;
    }
  }
  return false;
}

bool CVXSolver::configurationComputeOnly(const Eigen::VectorXd& sample, Eigen::VectorXd& result)
{
  if(!start_tree_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing initial configuration");
    return false;
  }
  fproblem_.newProblem();
  Eigen::Array2Xd bounds = fproblem_.getBounds();
  Eigen::ArrayXd init(bounds.cols() - dof_);
  for(int idx=dof_; idx < bounds.cols(); idx++)
  {
    init(idx - dof_) = (bounds(0,idx) + bounds(1,idx))/2.0;
  }
  Eigen::VectorXd formation_config(configuration_bounds_.cols());
  formation_config.head(dof_) = sample;
  formation_config.tail(formation_config.rows() - dof_) = init;
  fproblem_.setVariables(formation_config);
  if(!fproblem_.computePolyhedron())
  {
    RCLCPP_ERROR(node_->get_logger(), "Cannot compute polyhedron in the specified configuration");
    return false;
  }
  fproblem_.setGoal(formation_config);
  if(!fproblem_.solve(result))
  {
    RCLCPP_ERROR(node_->get_logger(), "Problems during configuration");
    return false;
  }
  return true;
}

void CVXSolver::resetProblem()
{
  goal_node_.reset();
  start_tree_.reset();
  poly_list_.clear(); // NOTE: unique_ptr instead of shared_ptr ??
  fproblem_.newProblem();
  solved_=false;
}

bool CVXSolver::getPolyhedronFromPoint(const Eigen::VectorXd& c, PolyhedronContainerPtr& return_poly)
{
  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::getPolyhedronFromPoint()");
//  int sampler_dof = sampler_->getDimension();
  if(c.rows() != sampler_->getDimension())
  {
    RCLCPP_ERROR(node_->get_logger(), "Dimensions of polyhedron seed is not that of the problem!");
    throw std::runtime_error("Dimensions of polyhedron seed is not that of the problem!");
  }

  iris::IRISProblem prb(dim_);
  prb.setSeedPoint(c.head(dim_));
  for(std::pair<std::string, std::vector<Eigen::Matrix2d>> obstacle : obstacles_)
  {
    for(Eigen::Matrix2d m : obstacle.second)
    {
      prb.addObstacle(m);
    }
  }
  iris::IRISOptions opt;
  opt.require_containment = true;
  iris::IRISRegion region;
  try
  {
    region = iris::inflate_region(prb, opt);
  }
  catch(const iris::InitialPointInfeasibleError& ex)
  {
    RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::getPolyhedronFromPoint() -> No Polyhedron detected");
    return false;
  }
  iris::Polyhedron poly = region.polyhedron;
  RCLCPP_DEBUG(node_->get_logger(), "~CVXSolver::getPolyhedronFromPoint()");
  return_poly = std::make_shared<PolyhedronContainer>(poly);
  return true;
}

bool CVXSolver::getPolyhedronFromNode(const NodePtr& node, PolyhedronContainerPtr& poly, bool attach)
{
  RCLCPP_DEBUG(node_->get_logger(), "CVXSolver::getPolyhedronFromNode()");
  bool poly_ok = getPolyhedronFromPoint(node->getConfiguration().head(dim_), poly);
  if(attach) poly->addNode(node);
  return poly_ok;
}

std::map<std::string, std::vector<Eigen::Vector3d>> CVXSolver::getConfigFromPath(const PathPtr& path)
{
  std::vector<pathplan::NodePtr> path_nodes = path->getNodes();

  using PathPosesVector = std::map<std::string, std::vector<Eigen::Vector3d>>;
  PathPosesVector ret;
  for (auto& [rname, poses] : ret)
  {
    poses.resize(path_nodes.size());
  }

  for(auto node_it = path_nodes.begin(); node_it!=path_nodes.end(); node_it++)
  {
    size_t idx = std::distance(path_nodes.begin(), node_it);
    Eigen::Vector3d center_formation_t_in_map = Eigen::Vector3d({map_node_to_config[*node_it].at(0),
                                                      map_node_to_config[*node_it].at(1),
                                                      0});
    Eigen::AngleAxisd center_formation_r_in_map = Eigen::AngleAxisd(map_node_to_config[*node_it].at(1), Eigen::Vector3d::UnitZ());

    for(auto rname_it = robot_loaded_.begin(); rname_it!=robot_loaded_.end(); ++rname_it)
    {
      size_t rname_idx = std::distance(robot_loaded_.begin(), rname_it);
      Eigen::Vector3d grasping_to_robot_in_robot = -(node_it->get()->getConfiguration()(dof_+rname_idx)) * Eigen::Vector3d::UnitX();
      Eigen::AngleAxisd rotation_around_grasping_point = Eigen::AngleAxisd(node_it->get()->getConfiguration()(dof_+robot_loaded_.size()+idx),Eigen::Vector3d::UnitZ());

      ret[*rname_it][idx] = center_formation_t_in_map + center_formation_r_in_map * (
            grasping_vertices_in_obj_.col(rname_idx) + rotation_around_grasping_point * (
              grasping_to_robot_in_robot
              ));
    }
  }
  return ret;

}

TreeSolverPtr CVXSolver::clone(const MetricsPtr &metrics, const CollisionCheckerPtr &checker, const SamplerPtr &sampler)
{
  CVXSolverPtr new_solver = std::make_shared<CVXSolver>(metrics,checker,sampler);
  new_solver->config(node_);
  return new_solver;
}

} // pathplan
