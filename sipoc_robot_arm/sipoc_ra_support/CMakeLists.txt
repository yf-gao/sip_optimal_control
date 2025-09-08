cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

project(sipoc_ra_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(octomap REQUIRED)

add_executable(stl2octomap_generation src/stl2octomap_generation.cpp)
target_include_directories(stl2octomap_generation PRIVATE ${coal_INCLUDE_DIRS})
target_link_libraries(
  stl2octomap_generation
  Eigen3::Eigen  octomap CGAL)
ament_target_dependencies(stl2octomap_generation ament_index_cpp)


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

## for correct libraries locations across platforms
include(GNUInstallDirs)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.bt"
  PATTERN "*.stl"
  PATTERN "*.dae"
)


install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.urdf"
)

ament_package()
