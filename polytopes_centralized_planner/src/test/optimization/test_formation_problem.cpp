#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <cvx_motion_planning/optimization/variable_set.h>
#include <cvx_motion_planning/optimization/constraints_set.h>
#include <cvx_motion_planning/optimization/cost_term.h>
#include <cvx_motion_planning/optimization/formation_problem.h>
#include <iris/iris.h>

#include <memory>

//#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <eigen_conversions/eigen_msg.h>
#include <rosparam_utilities/rosparam_utilities.h>

#include <cmath>

iris::Polyhedron computePolyhedron(const Eigen::VectorXd& seed,
                                   const std::vector<Eigen::Matrix2Xd>& obs)
{
  //std::cout<< "-- COMPUTE POLYHEDRON";
  iris::IRISProblem prb(2);
  prb.setSeedPoint(seed);
  for (const Eigen::Matrix2Xd& ob : obs)
  {
    prb.addObstacle(ob);
  }

  iris::IRISOptions opt;
  opt.require_containment = true;
  iris::IRISRegion region = iris::inflate_region(prb, opt);
  return region.getPolyhedron();
}

//Eigen::Vector3d get_vertex_in_world(const Eigen::Vector3d& object_center_and_theta,
//                                       const double arm_length,
//                                       const double theta_robot,
//                                       const Eigen::Vector3d& robot_in_robot,
//                                       const Eigen::Vector3d& grasp_in_obj,
//                                       const Eigen::Vector3d& center_to_grasping_in_robot
//                                       )
//{
//  double theta = object_center_and_theta(2);
//  Eigen::Matrix3d rot;
//  rot <<  std::cos(theta), -std::sin(theta), 0,
//          std::sin(theta),  std::cos(theta), 0,
//          0,0,1;
//  Eigen::Matrix3d mrot;
//  mrot << std::cos(theta_robot), -std::sin(theta_robot), 0,
//          std::sin(theta_robot),  std::cos(theta_robot), 0,
//          0,0,1;
////  Eigen::Vector3d center_to_grasping_in_robot;
////  center_to_grasping_in_robot << arm_length*std::cos(theta_robot),
////                                 arm_length*std::sin(theta_robot),
////                                 0;
//  Eigen::Vector3d t;
//  t << object_center_and_theta(0), object_center_and_theta(1), 0;
//  return t + rot * (grasp_in_obj + mrot*(robot_in_robot - arm_length*center_to_grasping_in_robot));
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_optimization");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/formation_result", 1000);
  ros::Rate loop_rate(10);
//  geometry_msgs::Point32 dummy_point_msg;
//  geometry_msgs::Polygon polygon_msg;
  std_msgs::Float64MultiArray result_msg;
  std::string possible_error;

  FormationProblem fproblem = FormationProblem();
  // ===========================================================
  // Variable SET
  std::cout << "VARIABLE SET" << std::endl;
  std::vector<std::string> robots_names;
  if(!rosparam_utilities::get("/robot_vertices/robot_to_load", robots_names, possible_error))
  {
    ROS_FATAL_STREAM(possible_error);
    assert(false);
  }
  ROS_WARN_STREAM(robots_names.at(0) << " " << robots_names.at(1));
  int n_manip = robots_names.size();

  Eigen::VectorXd x_init(3+n_manip*2);

  if(!rosparam_utilities::get("init", x_init, possible_error))
  {
    ROS_FATAL_STREAM(possible_error);
    assert(false);
  }
  Eigen::MatrixXd var_bounds;

  if(!rosparam_utilities::get("bounds", var_bounds, possible_error))
  {
    ROS_FATAL_STREAM(possible_error);
    assert(false);
  }
  ROS_WARN_STREAM("VAR_BOUNDS:\n" << var_bounds);
  var_bounds.transposeInPlace();
  fproblem.setVariables(x_init, var_bounds);

  // ===========================================================
  // Constraint SET
  std::cout << "CONSTRAINT SET" << std::endl;
  std::vector<Eigen::Matrix2Xd> obs;
  std::vector<std::vector<double>> object_input;

  Eigen::Matrix2Xd single_obstacle(2,2);
  single_obstacle.setZero();

  // BEGIN Parameters: Obstacles
  if(!rosparam_utilities::get("cube_1",object_input,possible_error))
    ROS_FATAL_STREAM("MISSING-PARAMETER");
  for (int idx = 0; idx < object_input.size(); idx++ )
  {
    Eigen::Vector2d vv = Eigen::Map<Eigen::VectorXd>(&object_input.at(idx)[0], 2);
    single_obstacle.col(0) = vv;
    vv = Eigen::Map<Eigen::VectorXd>(&object_input.at((idx+1) % object_input.size())[0], 2);
    single_obstacle.col(1) = vv;
    obs.push_back(single_obstacle);
  }

  if(!rosparam_utilities::get("cube_2",object_input,possible_error))
    ROS_FATAL_STREAM("MISSING-PARAMETER");
  for (int idx = 0; idx < object_input.size(); idx++ )
  {
    Eigen::Vector2d vv = Eigen::Map<Eigen::VectorXd>(&object_input.at(idx)[0], 2);
    single_obstacle.col(0) = vv;
    vv = Eigen::Map<Eigen::VectorXd>(&object_input.at((idx+1) % object_input.size())[0], 2);
    single_obstacle.col(1) = vv;
    obs.push_back(single_obstacle);
  }

  if(!rosparam_utilities::get("scene_bound",object_input,possible_error))
    ROS_FATAL_STREAM("MISSING-PARAMETER");
  for (int idx = 0; idx < object_input.size(); idx++ )
  {
    Eigen::Vector2d vv = Eigen::Map<Eigen::VectorXd>(&object_input.at(idx)[0], 2);
    single_obstacle.col(0) = vv;
    vv = Eigen::Map<Eigen::VectorXd>(&object_input.at((idx+1) % object_input.size())[0], 2);
    single_obstacle.col(1) = vv;
    obs.push_back(single_obstacle);
  }
  // END Obstacles

  Eigen::MatrixXd vertices_of_obj_in_obj; // #
  if(!rosparam_utilities::get("/object_vertices", vertices_of_obj_in_obj, possible_error))
  {
    //ROS_FATAL(possible_error);
    assert(false);
  }
  vertices_of_obj_in_obj.transposeInPlace();

  std::vector<Eigen::Matrix3Xd> vertices_of_robot_in_robot; // #
  Eigen::MatrixXd v_robot;
  for(auto& name : robots_names)
  {
    if(!rosparam_utilities::get("/robot_vertices/"+name, v_robot, possible_error))
    {
      //ROS_FATAL(possible_error);
      assert(false);
    }
    v_robot.transposeInPlace();
    vertices_of_robot_in_robot.push_back(v_robot);
    ROS_WARN_STREAM(v_robot);
  }

  int number_of_vertices = vertices_of_obj_in_obj.cols();
  for (auto v : vertices_of_robot_in_robot)
  {
    number_of_vertices += v.cols();
  }

  Eigen::MatrixXd grasp_in_obj; // #
  if(!rosparam_utilities::get("grasping_vertices_in_obj",grasp_in_obj,possible_error))
  {
    //ROS_FATAL(possible_error);
    assert(false);
  }
  grasp_in_obj.transposeInPlace();
  Eigen::Vector2d poly_seed;
  poly_seed << x_init(0),
               x_init(1);

  Eigen::MatrixXd center_to_grapsing_in_robot;
  if(!rosparam_utilities::get("grasping_from_center_in_robot", center_to_grapsing_in_robot, possible_error))
  {
    //ROS_FATAL(possible_error);
    assert(false);
  }
  center_to_grapsing_in_robot.transposeInPlace();

  for(int idx = 0; idx < center_to_grapsing_in_robot.cols(); idx++)
    center_to_grapsing_in_robot.col(idx) = center_to_grapsing_in_robot.col(idx)/center_to_grapsing_in_robot.col(idx).norm();

  ROS_WARN_STREAM(center_to_grapsing_in_robot);

  fproblem.setObjVertices(vertices_of_obj_in_obj, grasp_in_obj);
  fproblem.setRobotsVertices(vertices_of_robot_in_robot, center_to_grapsing_in_robot);
  fproblem.setObstacles(obs);
  if(!fproblem.computePolyhedron()) assert(false);
  std::cout << "-- POST CONTRAINT DEFINITION" << std::endl;
  // ===========================================================
  // Cost TERM
  std::cout << "COST TERM" << std::endl;
  Eigen::VectorXd goal;
  if(!rosparam_utilities::get("goal", goal, possible_error))
  {
    //ROS_FATAL(possible_error);
    assert(false);
  }
  //goal.transposeInPlace();
  ROS_WARN_STREAM(goal);
  std::cout << "END COST TERM" << std::endl;
  fproblem.setGoal(goal);
  // ===========================================================
  Eigen::VectorXd x;
  fproblem.solve(x, true);
  std::cout << "Solution: " << x.transpose() << std::endl;
  result_msg.data.clear();
  result_msg.layout.dim.resize(1);
  result_msg.layout.dim[0].label = "rows";
  result_msg.layout.dim[0].size = x.rows();
  result_msg.layout.dim[0].stride = x.rows();
  for(int idx = 0; idx < x.rows(); idx++)
    result_msg.data.push_back(x(idx));
  pub.publish(result_msg);
  loop_rate.sleep();
//  while(true)
//  {
//    loop_rate.sleep();
//  }
}
