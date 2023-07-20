cmake_minimum_required(VERSION 3.8)
project(polytopes_centralized_planner)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# ROS packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(graph_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav2_msgs REQUIRED) # per action

# Custom msgs
find_package(polytopes_centralized_planner_msgs REQUIRED)

# Other packages
find_package(Eigen3 REQUIRED)
find_package(iris REQUIRED)
find_package(ifopt REQUIRED)

# Libreria
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/solvers/polytopes_solver.cpp
  src/${PROJECT_NAME}/optimization/formation_problem.cpp
  src/${PROJECT_NAME}/formation.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  IRIS
  ifopt
  graph_core
  rclcpp
  moveit_core
  geometry_msgs
  moveit_msgs
  tf2 
  tf2_ros
  tf2_eigen
  polytopes_centralized_planner_msgs
)
ament_export_targets(${PROJECT_NAME}Target HAS_LIBRARY_TARGET)

#################### centralized_planner_node ##############################
add_executable(centralized_planner_node
  src/centralized_planner_node.cpp
)
target_include_directories(centralized_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(centralized_planner_node
  Eigen3
  rclcpp
  rclcpp_lifecycle
  graph_core
  nav2_msgs
  geometry_msgs
  rclcpp_action
  rclcpp_components
  tf2
  tf2_eigen
  IRIS
  )
# Necessario perché ament_target_dependencies() non funziona con target definiti nello stesso CMakeLists
target_link_libraries(centralized_planner_node
  ${PROJECT_NAME}
)
############################################################################

# Python package install
ament_python_install_package(${PROJECT_NAME})


## INSTALL

# installa directory di include
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION include
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

# # Installa target
# install(TARGETS test_solver
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# Installa libreria
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Target
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# Installa nodo
install(TARGETS centralized_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# installa python scripts
install(
  PROGRAMS scripts/simplify_map.py
  DESTINATION lib/${PROJECT_NAME}
)

# installa directory di launch
install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
