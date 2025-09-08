cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

project(sipoc_ra_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_TESTING ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(manif REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pinocchio REQUIRED)
find_package(GSL REQUIRED)

include_directories(
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(sipoc_ra_utils_utils SHARED
  src/utils.cpp
)
target_include_directories(sipoc_ra_utils_utils PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(sipoc_ra_utils_utils PRIVATE ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS})
target_link_libraries(
  sipoc_ra_utils_utils Eigen3::Eigen pinocchio::pinocchio MANIF::manif)

add_library(sia20d_car_seat_velocity_profile_optimizer
    src/sia20d_car_seat_velocity_profile_optimizer.cpp
    src/spline_wrapper_8dim.cpp
    src/spline_fitter_8dim.cpp
    src/time_optimal_velocity.cpp)
target_include_directories(sia20d_car_seat_velocity_profile_optimizer PUBLIC
    ${sipoc_ra_utils_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS})
target_link_libraries(
  sia20d_car_seat_velocity_profile_optimizer PUBLIC
  Eigen3::Eigen MANIF::manif  GSL::gsl
)

if(BUILD_TESTING)
  add_subdirectory(tests)
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

include(GNUInstallDirs)

install(DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)


install(
    TARGETS sipoc_ra_utils_utils sia20d_car_seat_velocity_profile_optimizer
    EXPORT sipoc_ra_utils_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
ament_export_targets(sipoc_ra_utils_targets HAS_LIBRARY_TARGET)

ament_package()
