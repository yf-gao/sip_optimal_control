cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

project(sipoc_ra_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_TESTING ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(coal REQUIRED)
find_package(CGAL REQUIRED)
find_package(manif REQUIRED)
find_package(GSL REQUIRED)
find_package(sipoc_ra_support REQUIRED)
find_package(sipoc_ra_utils REQUIRED)
if(NOT OpenVDB_DIR)
    set(OpenVDB_DIR "${CMAKE_INSTALL_PREFIX}/../OpenVDB/lib/cmake/OpenVDB")
endif()
list(APPEND CMAKE_MODULE_PATH "${OpenVDB_DIR}")
find_package(OpenVDB REQUIRED)

set(COAL_HAS_OCTOMAP TRUE)
string(REPLACE "." ";" VERSION_LIST ${octomap_VERSION})
list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)

if(DEFINED ACADOS_INSTALL_DIR)
    set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
    set(ACADOS_LIB_DIR "${ACADOS_INSTALL_DIR}/lib")
else()
    set(ACADOS_INCLUDE_DIR "$ENV{ACADOS_SOURCE_DIR}/include")
    set(ACADOS_LIB_DIR "$ENV{ACADOS_SOURCE_DIR}/lib")
endif()
set(ACADOS_GENERATED_CODE_PLANNING_DIRS acados_generated_code/c_generated_code_sipoc_ra_planning)
set(ACADOS_GENERATED_CODE_MPC_DIRS acados_generated_code/c_generated_code_sipoc_ra_mpc)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(sipoc_ra_collision_constraint_manager SHARED
        src/collision_constraint_manager.cpp
        src/pinocchio_kinematics.cpp
        src/coal_collision_detection.cpp
        src/openvdb_signed_distance_field.cpp)
target_include_directories(sipoc_ra_collision_constraint_manager PUBLIC
        ${sipoc_ra_utils_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS} ${coal_INCLUDE_DIRS} ${OpenVDB_INCLUDE_DIRS} )
target_link_libraries(
  sipoc_ra_collision_constraint_manager PUBLIC
  sipoc_ra_utils::sipoc_ra_utils_utils
  Eigen3::Eigen pinocchio::pinocchio MANIF::manif coal::coal octomap openvdb CGAL)
target_link_directories(sipoc_ra_collision_constraint_manager PUBLIC ${OpenVDB_LIBRARY_DIRS})
target_compile_definitions(sipoc_ra_collision_constraint_manager PUBLIC COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP FCL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})


add_library(sipoc_ra_car_seat_planning_solver SHARED
        src/car_seat_trajectory_solver.cpp
        "include/${ACADOS_GENERATED_CODE_PLANNING_DIRS}/acados_solver_robot_joints.c"
        "include/${ACADOS_GENERATED_CODE_PLANNING_DIRS}/robot_joints_model/robot_joints_expl_ode_fun.c"
        "include/${ACADOS_GENERATED_CODE_PLANNING_DIRS}/robot_joints_model/robot_joints_expl_vde_adj.c"
        "include/${ACADOS_GENERATED_CODE_PLANNING_DIRS}/robot_joints_model/robot_joints_expl_vde_forw.c")
target_include_directories(sipoc_ra_car_seat_planning_solver PUBLIC
        ${sipoc_ra_utils_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS} ${coal_INCLUDE_DIRS} ${OpenVDB_INCLUDE_DIRS})
target_include_directories(sipoc_ra_car_seat_planning_solver PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(sipoc_ra_car_seat_planning_solver PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(sipoc_ra_car_seat_planning_solver PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
ament_target_dependencies(sipoc_ra_car_seat_planning_solver rclcpp ament_index_cpp)
target_link_libraries(
  sipoc_ra_car_seat_planning_solver
  sipoc_ra_collision_constraint_manager
  sipoc_ra_utils::sipoc_ra_utils_utils
  Eigen3::Eigen MANIF::manif)
target_link_libraries(sipoc_ra_car_seat_planning_solver "${ACADOS_LIB_DIR}/libacados.so")
target_compile_definitions(sipoc_ra_car_seat_planning_solver PUBLIC COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP FCL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
target_compile_definitions(sipoc_ra_car_seat_planning_solver PRIVATE SIPOC_RA_SOLVER__SIA20D_PLANNING)

add_library(sipoc_ra_car_seat_ocp_solver SHARED
        src/car_seat_trajectory_solver.cpp
        "include/${ACADOS_GENERATED_CODE_MPC_DIRS}/acados_solver_robot_joints.c"
        "include/${ACADOS_GENERATED_CODE_MPC_DIRS}/robot_joints_model/robot_joints_expl_ode_fun.c"
        "include/${ACADOS_GENERATED_CODE_MPC_DIRS}/robot_joints_model/robot_joints_expl_vde_adj.c"
        "include/${ACADOS_GENERATED_CODE_MPC_DIRS}/robot_joints_model/robot_joints_expl_vde_forw.c")
target_include_directories(sipoc_ra_car_seat_ocp_solver PUBLIC ${sipoc_ra_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS} ${coal_INCLUDE_DIRS} ${OpenVDB_INCLUDE_DIRS} )
target_include_directories(sipoc_ra_car_seat_ocp_solver PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(sipoc_ra_car_seat_ocp_solver PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(sipoc_ra_car_seat_ocp_solver PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
ament_target_dependencies(sipoc_ra_car_seat_ocp_solver rclcpp ament_index_cpp sipoc_ra_support)
target_link_libraries(
  sipoc_ra_car_seat_ocp_solver
  sipoc_ra_collision_constraint_manager
  sipoc_ra_utils::sipoc_ra_utils_utils
  Eigen3::Eigen octomap  MANIF::manif)
target_link_libraries(sipoc_ra_car_seat_ocp_solver "${ACADOS_LIB_DIR}/libacados.so")
target_link_directories(sipoc_ra_car_seat_ocp_solver PUBLIC ${OpenVDB_LIBRARY_DIRS})
target_compile_definitions(sipoc_ra_car_seat_ocp_solver PUBLIC COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP FCL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
target_compile_definitions(sipoc_ra_car_seat_ocp_solver PRIVATE SIPOC_RA_SOLVER__SIA20D_MPC)

# Add example subfolder
add_subdirectory(examples)

# Add test subfolder
if(BUILD_TESTING)
add_subdirectory(test)
endif()

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

#################
# INSTALLATION  #
#################

## for correct libraries locations across platforms
include(GNUInstallDirs)

install(
  DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)
ament_export_include_directories(include)

install(
  TARGETS sipoc_ra_collision_constraint_manager sipoc_ra_car_seat_planning_solver sipoc_ra_car_seat_ocp_solver
  EXPORT sipoc_ra_solver_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
ament_export_targets(sipoc_ra_solver_targets HAS_LIBRARY_TARGET)

ament_package()
