add_executable(example_sia20d_car_seat_planner sia20d_car_seat_planner.cpp)
target_include_directories(example_sia20d_car_seat_planner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(example_sia20d_car_seat_planner PRIVATE ${EIGEN3_INCLUDE_DIR}  ${pinocchio_INCLUDE_DIRS}  ${sipoc_ra_utils_INCLUDE_DIRS}  ${manif_INCLUDE_DIRS} ${coal_INCLUDE_DIRS} ${OpenVDB_INCLUDE_DIRS})
target_include_directories(example_sia20d_car_seat_planner PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(example_sia20d_car_seat_planner PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(example_sia20d_car_seat_planner PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
target_link_libraries(
  example_sia20d_car_seat_planner
  sipoc_ra_car_seat_planning_solver
  Eigen3::Eigen  pinocchio::pinocchio  MANIF::manif coal::coal openvdb
  )
ament_target_dependencies(example_sia20d_car_seat_planner ament_index_cpp)
target_compile_definitions(example_sia20d_car_seat_planner PUBLIC COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP FCL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
target_compile_definitions(example_sia20d_car_seat_planner PRIVATE SIPOC_RA_SOLVER__SIA20D_PLANNING)

add_executable(example_sia20d_car_seat_mpc sia20d_car_seat_mpc.cpp)
target_include_directories(example_sia20d_car_seat_mpc PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(example_sia20d_car_seat_mpc PRIVATE ${EIGEN3_INCLUDE_DIR}${pinocchio_INCLUDE_DIRS}  ${sipoc_ra_utils_INCLUDE_DIRS} ${manif_INCLUDE_DIRS} ${coal_INCLUDE_DIRS} ${OpenVDB_INCLUDE_DIRS})
target_include_directories(example_sia20d_car_seat_mpc PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(example_sia20d_car_seat_mpc PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(example_sia20d_car_seat_mpc PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
target_link_libraries(
  example_sia20d_car_seat_mpc
  sipoc_ra_car_seat_ocp_solver sipoc_ra_utils::sia20d_car_seat_velocity_profile_optimizer
  Eigen3::Eigen  pinocchio::pinocchio  MANIF::manif coal::coal openvdb
  )
ament_target_dependencies(example_sia20d_car_seat_mpc ament_index_cpp)
target_compile_definitions(example_sia20d_car_seat_mpc PUBLIC COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP FCL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
target_compile_definitions(example_sia20d_car_seat_mpc PRIVATE SIPOC_RA_SOLVER__SIA20D_MPC)
