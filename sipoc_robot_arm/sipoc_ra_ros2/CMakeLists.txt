cmake_minimum_required(VERSION 3.8)

project(sipoc_ra_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(CGAL REQUIRED)
find_package(coal REQUIRED)
find_package(manif REQUIRED)
find_package(GSL REQUIRED)
find_package(sipoc_ra_utils REQUIRED)
find_package(sipoc_ra_solver REQUIRED)

if(DEFINED ACADOS_INSTALL_DIR)
    set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
else()
    set(ACADOS_INCLUDE_DIR "$ENV{ACADOS_SOURCE_DIR}/include")
endif()

add_executable(sia20d_car_seat_planner_node src/sia20d_car_seat_planner_node.cpp)
target_include_directories(sia20d_car_seat_planner_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(sia20d_car_seat_planner_node PRIVATE
       ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS}
       ${sipoc_ra_utils_INCLUDE_DIRS} ${sipoc_ra_solver_INCLUDE_DIRS})
target_include_directories(sia20d_car_seat_planner_node PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(sia20d_car_seat_planner_node PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(sia20d_car_seat_planner_node PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
ament_target_dependencies(sia20d_car_seat_planner_node rclcpp visualization_msgs tf2_ros)
target_link_libraries(
  sia20d_car_seat_planner_node
  sipoc_ra_utils::sipoc_ra_utils_utils
  sipoc_ra_solver::sipoc_ra_car_seat_planning_solver
  Eigen3::Eigen
)
target_compile_definitions(sia20d_car_seat_planner_node PRIVATE SIPOC_RA_SOLVER__SIA20D_PLANNING)


add_executable(sia20d_car_seat_mpc_node src/sia20d_car_seat_mpc_node.cpp)
target_include_directories(sia20d_car_seat_mpc_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(sia20d_car_seat_mpc_node PRIVATE
        ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS}
        ${sipoc_ra_utils_INCLUDE_DIRS} ${sipoc_ra_solver_INCLUDE_DIRS})
target_include_directories(sia20d_car_seat_mpc_node PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(sia20d_car_seat_mpc_node PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(sia20d_car_seat_mpc_node PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
ament_target_dependencies(sia20d_car_seat_mpc_node rclcpp visualization_msgs tf2_ros)
target_link_libraries(
  sia20d_car_seat_mpc_node
  sipoc_ra_utils::sipoc_ra_utils_utils
  sipoc_ra_utils::sia20d_car_seat_velocity_profile_optimizer
  sipoc_ra_solver::sipoc_ra_car_seat_ocp_solver
  Eigen3::Eigen
)
target_compile_definitions(sia20d_car_seat_mpc_node PRIVATE SIPOC_RA_SOLVER__SIA20D_MPC)

## for correct libraries locations across platforms
include(GNUInstallDirs)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS sia20d_car_seat_planner_node sia20d_car_seat_mpc_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
