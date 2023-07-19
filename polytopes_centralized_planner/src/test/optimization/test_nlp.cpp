#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <cvx_motion_planning/optimization/variable_set.h>
#include <cvx_motion_planning/optimization/constraints_set.h>
#include <cvx_motion_planning/optimization/cost_term.h>
#include <iris/iris.h>

#include <memory>

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <eigen_conversions/eigen_msg.h>

#include <cmath>

//iris::Polyhedron computePolyhedron(const Eigen::VectorXd& seed,
//                                   const std::vector<Eigen::Matrix2Xd>& obs,
//                                   const std::vector<Eigen::Matrix3Xd>& robot_verticies,
//                                   const Eigen::Matrix3Xd& obj_verticies)
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
//  for (int idx = 0; idx < obj_verticies.cols(); idx++)
//  {
//    opt.required_containment_points.push_back(obj_verticies.topRows<2>().col(idx));
//  }
//  for (int idx = 0; idx < robot_verticies.size(); idx++)
//  {
//    for (int jdx = 0; jdx < robot_verticies.at(idx).cols(); jdx++)
//    {
//      opt.required_containment_points.push_back(robot_verticies.at(idx).topRows<2>().col(jdx));
//    }
//  }
  iris::IRISRegion region = iris::inflate_region(prb, opt);
  return region.getPolyhedron();
}

Eigen::Vector3d get_vertex_in_world(const Eigen::Vector3d& object_center_and_theta,
                                       const double arm_length,
                                       const double theta_robot,
                                       const Eigen::Vector3d& robot_in_robot,
                                       const Eigen::Vector3d& grasp_in_obj
                                       )
{
  double theta = object_center_and_theta(2);
  Eigen::Matrix3d rot;
  rot <<  std::cos(theta), -std::sin(theta), 0,
          std::sin(theta),  std::cos(theta), 0,
          0,0,1;
  Eigen::Matrix3d mrot;
  mrot << std::cos(theta_robot), -std::sin(theta_robot), 0,
          std::sin(theta_robot),  std::cos(theta_robot), 0,
          0,0,1;
  Eigen::Vector3d center_to_grasping_in_robot;
  center_to_grasping_in_robot << arm_length*std::cos(theta_robot),
                                 arm_length*std::sin(theta_robot),
                                 0;
  Eigen::Vector3d t;
  t << object_center_and_theta(0), object_center_and_theta(1), 0;
  return t + rot * (grasp_in_obj + mrot*(robot_in_robot - center_to_grasping_in_robot));
}

int main(int argc, char **argv)
{
//  ros::init(argc, argv, "test_optimization");
//  ros::NodeHandle nh;

//  ros::Publisher pub = nh.advertise<geometry_msgs::Polygon>("/polygon_points", 1000);
//  ros::Rate loop_rate(10);
//  Eigen::Vector3d dummy_for_msg;
//  geometry_msgs::Point32 dummy_point_msg;
//  geometry_msgs::Polygon polygon_msg;

  std::string name_varset = "var_set";
  std::string name_constrset = "constr_set";
  std::string name_cost = "cost_term";

  ifopt::Problem nlp;
  Eigen::VectorXd obstacles;
  const int n_manip = 2;
  Eigen::Matrix2d scene;
  // ===========================================================
  // Variable SET
  std::cout << "VARIABLE SET" << std::endl;
  Eigen::VectorXd x_init(3+n_manip*2);
  //      [tx, ty, theta,  a1,  a2, phi1, phi2]
  x_init << 3.0, 2.0, M_PI, 0.5, 0.5, 0.0, 0.0; // init_values
  Eigen::Matrix2Xd var_bounds(2, 3+n_manip*2);
  //           [tx, ty, theta,                                    a1,  a2,  phi1, phi2]
  var_bounds << 0,  0,  -std::numeric_limits<double>::infinity(), 0.1, 0.1, 0,    0,
                5,  5,   std::numeric_limits<double>::infinity(), 1.0, 1.0, 0,    0;

  std::shared_ptr<FormationVar> var_ptr = std::make_shared<FormationVar>(name_varset, x_init, var_bounds);
  nlp.AddVariableSet(var_ptr);

  // ===========================================================
  // Constraint SET
  std::cout << "CONSTRAINT SET" << std::endl;
  std::vector<Eigen::Matrix2Xd> obs;
  Eigen::Matrix2Xd single_ob(2,2);
  // Obj 1
  single_ob << 0, 1,
               0, 0;
  obs.push_back(single_ob);
//  for (int idx = 0; idx < single_ob.cols(); idx++)
//  {
//    dummy_for_msg << single_ob(idx,0), single_ob(idx,1), 0;
//    tf::pointEigenToMsg(dummy_for_msg, dummy_point_msg); // cast ??
//    polygon_msg.points.push_back(dummy_point_msg);
//  }
  //pub.publish(polygon_msg);
  single_ob << 1, 1,
               0, 1;
  obs.push_back(single_ob);
  single_ob << 1, 0,
               1, 1;
  obs.push_back(single_ob);
  single_ob << 0, 0,
               1, 0;
  obs.push_back(single_ob);
  // Obj 2
  single_ob << 5, 4,
               1, 1;
  obs.push_back(single_ob);
  single_ob << 4, 4,
               1, 4;
  obs.push_back(single_ob);
  single_ob << 4, 5,
               4, 4;
  obs.push_back(single_ob);
  single_ob << 5, 5,
               4, 1;
  obs.push_back(single_ob);
  // BOX
  single_ob << 0, 0,
               0, 5;
  obs.push_back(single_ob);
  single_ob << 0, 5,
               5, 5;
  obs.push_back(single_ob);
  single_ob << 5, 5,
               5, 0;
  obs.push_back(single_ob);
  single_ob << 5, 0,
               0, 0;
  obs.push_back(single_ob);

  Eigen::Matrix3Xd vertices_of_obj_in_obj(3,4); // #
  vertices_of_obj_in_obj << -0.5, 0.5, 0.5, -0.5,
                            0.5, 0.5, -0.5, -0.5,
                              1,   1,    1,    1;
//  Eigen::Matrix3Xd vertices_of_obj_in_world;
//  for(int idx=0; idx < vertices_of_obj_in_obj.cols(); idx++)
//    vertices_of_obj_in_world.col(idx) = get_vertex_in_world(x_init.segment<3>(0),0, 0, vertices_of_obj_in_obj.col(idx), Eigen::Vector3d::Zero());

  std::vector<Eigen::Matrix3Xd> vertices_of_robot_in_robot; // #
  Eigen::Matrix3Xd v_robot(3,4);
  v_robot << -0.25, 0.25, 0.25, -0.25,
              0.25, 0.25, -0.25, -0.25,
                 1,   1,    1,    1;
  vertices_of_robot_in_robot.push_back(v_robot);
  v_robot << -0.25, 0.25, 0.25, -0.25,
              0.25, 0.25, -0.25, -0.25,
                 1,   1,    1,    1;
  vertices_of_robot_in_robot.push_back(v_robot);

  int number_of_vertices = vertices_of_obj_in_obj.cols();
  for (auto v : vertices_of_robot_in_robot)
  {
    number_of_vertices += v.cols();
  }

  Eigen::Matrix3Xd grasp_in_obj(3,2); // #
  grasp_in_obj << -0.5, 0.5,
                     0,   0,
                     1,   1;

  Eigen::Vector2d poly_seed;
  poly_seed << x_init(0),
               x_init(1);

  iris::Polyhedron poly = computePolyhedron(poly_seed, obs); // #

  std::cout << "-- POLYHEDRON: A = " << std::endl << poly.getA() << "\nB = " << std::endl << poly.getB() << std::endl;

  Eigen::VectorXd center_to_grapsing_in_robot;
  center_to_grapsing_in_robot << 1, 0;
  center_to_grapsing_in_robot = center_to_grapsing_in_robot/center_to_grapsing_in_robot.norm();

  std::cout << "HERE";
  std::shared_ptr<FormationConstr> constr_set =
      std::make_shared<FormationConstr>(name_constrset,
                                        name_varset,
                                        number_of_vertices * poly.getNumberOfConstraints(),
                                        vertices_of_obj_in_obj,
                                        vertices_of_robot_in_robot,
                                        grasp_in_obj,
                                        center_to_grapsing_in_robot,
                                        poly);
  std::cout << "-- POST CONTRAINT DEFINITION" << std::endl;
  nlp.AddConstraintSet(constr_set);
  // ===========================================================
  // Cost TERM
  std::cout << "COST TERM" << std::endl;
  Eigen::Vector2d goal;
  goal << 3, 2;
  std::shared_ptr<FormationCost> cost_term = std::make_shared<FormationCost>(name_cost, name_varset, goal);
  std::cout << "END COST TERM" << std::endl;
  nlp.AddCostSet(cost_term);
  // ===========================================================
  nlp.PrintCurrent();

  // Solver SETUP
  std::cout << "IPOPT SOLVER" << std::endl;
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("jacobian_approximation","finite-difference-values"); // "exact"
  ipopt.SetOption("linear_solver","mumps");

  // SOLVE
  std::cout << "SOLVING" << std::endl;
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << "Solution: " << x.transpose() << std::endl;
}
