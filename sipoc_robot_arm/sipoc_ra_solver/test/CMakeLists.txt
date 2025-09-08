add_executable(test_pinocchio_kinematics test_pinocchio_kinematics.cpp ../src/pinocchio_kinematics.cpp)
  target_include_directories(test_pinocchio_kinematics PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(test_pinocchio_kinematics PRIVATE ${EIGEN3_INCLUDE_DIR} ${pinocchio_INCLUDE_DIRS}  ${sipoc_ra_utils_INCLUDE_DIRS}  ${manif_INCLUDE_DIRS})
  target_link_libraries(
    test_pinocchio_kinematics
    Eigen3::Eigen pinocchio::pinocchio MANIF::manif
    sipoc_ra_utils::sipoc_ra_utils_utils)
ament_target_dependencies(test_pinocchio_kinematics ament_index_cpp)

add_executable(test_coal_collision_detection test_coal_collision_detection.cpp ../src/coal_collision_detection.cpp)
target_include_directories(test_coal_collision_detection PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(test_coal_collision_detection PRIVATE ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS} ${coal_INCLUDE_DIRS} ${sipoc_ra_utils_INCLUDE_DIRS}  ${pinocchio_INCLUDE_DIRS})
target_link_libraries(
  test_coal_collision_detection
  Eigen3::Eigen MANIF::manif coal::coal TBB::tbb)
target_compile_definitions(test_coal_collision_detection PUBLIC COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP FCL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
ament_target_dependencies(test_coal_collision_detection ament_index_cpp)

add_executable(test_openvdb_distance_field test_openvdb_signed_distance_field.cpp ../src/openvdb_signed_distance_field.cpp)
target_include_directories(test_openvdb_distance_field PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(test_openvdb_distance_field PRIVATE ${EIGEN3_INCLUDE_DIR} ${OpenVDB_INCLUDE_DIRS} )
target_link_libraries(
  test_openvdb_distance_field
  Eigen3::Eigen openvdb TBB::tbb)
target_link_directories(test_openvdb_distance_field PRIVATE ${OpenVDB_LIBRARY_DIRS})
ament_target_dependencies(test_openvdb_distance_field ament_index_cpp)

add_executable(test_collision_constraint_manager test_collision_constraint_manager.cpp ../src/collision_constraint_manager.cpp ../src/pinocchio_kinematics.cpp ../src/coal_collision_detection.cpp ../src/openvdb_signed_distance_field.cpp)
target_include_directories(test_collision_constraint_manager PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(test_collision_constraint_manager PRIVATE ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS}  ${coal_INCLUDE_DIRS} ${OpenVDB_INCLUDE_DIRS} ${sipoc_ra_utils_INCLUDE_DIRS})
target_link_libraries(
  test_collision_constraint_manager
  Eigen3::Eigen pinocchio::pinocchio MANIF::manif coal::coal openvdb TBB::tbb
  sipoc_ra_utils::sipoc_ra_utils_utils)
target_link_directories(test_collision_constraint_manager PRIVATE ${OpenVDB_LIBRARY_DIRS})
target_compile_definitions(test_collision_constraint_manager PUBLIC COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP FCL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
ament_target_dependencies(test_collision_constraint_manager ament_index_cpp)
