cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

project(sipoc_mr_nav2_controller)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/Modules)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(CLARABEL REQUIRED)
find_package(manif REQUIRED)
find_package(GSL REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(sipoc_mr_solver REQUIRED)
find_package(sipoc_mr_utils REQUIRED)
find_package(sipoc_ros2_interfaces REQUIRED)

nav2_package()

include_directories(
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

set(dependencies
  nav2_costmap_2d
  nav2_core
  nav2_util
  pluginlib
  rclcpp
  tf2
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  sipoc_mr_solver
  sipoc_mr_utils
  sipoc_ros2_interfaces
)

if(DEFINED ACADOS_INSTALL_DIR)
    set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
else()
    set(ACADOS_INCLUDE_DIR "$ENV{ACADOS_SOURCE_DIR}/include")
endif()
set(ACADOS_GENERATED_CODE_DIRS acados_generated_code/c_generated_code_sipoc_mr)

#########################################
# nominal capsule controller plugin #
#########################################

add_library(nav2_capsule_nominal_controller_plugin SHARED
        src/nav2_capsule_nominal_controller_plugin.cpp)

# prevent pluginlib from using boost
target_compile_definitions(nav2_capsule_nominal_controller_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(nav2_capsule_nominal_controller_plugin PRIVATE POLYGON_FLAG_VALUE=0)

target_include_directories(nav2_capsule_nominal_controller_plugin PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(nav2_capsule_nominal_controller_plugin PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(nav2_capsule_nominal_controller_plugin PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(nav2_capsule_nominal_controller_plugin PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(nav2_capsule_nominal_controller_plugin PRIVATE "${sipoc_mr_solver_INCLUDE_DIRS}/${ACADOS_GENERATED_CODE_DIRS}")
target_include_directories(nav2_capsule_nominal_controller_plugin PRIVATE ${sipoc_mr_utils_INCLUDE_DIRS})
target_include_directories(nav2_capsule_nominal_controller_plugin PRIVATE ${sipoc_mr_solver_INCLUDE_DIRS})
target_link_libraries(nav2_capsule_nominal_controller_plugin sipoc_mr_solver::capsule_nominal_controller)
ament_target_dependencies(nav2_capsule_nominal_controller_plugin ${dependencies})

#########################################
# nominal dilated polygon controller plugin #
#########################################

add_library(nav2_dilated_polygon_nominal_controller_plugin SHARED
        src/nav2_dilated_polygon_nominal_controller_plugin.cpp)

# prevent pluginlib from using boost
target_compile_definitions(nav2_dilated_polygon_nominal_controller_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(nav2_dilated_polygon_nominal_controller_plugin PRIVATE POLYGON_FLAG_VALUE=1)

target_include_directories(nav2_dilated_polygon_nominal_controller_plugin PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(nav2_dilated_polygon_nominal_controller_plugin PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(nav2_dilated_polygon_nominal_controller_plugin PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(nav2_dilated_polygon_nominal_controller_plugin PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(nav2_dilated_polygon_nominal_controller_plugin PRIVATE "${sipoc_mr_solver_INCLUDE_DIRS}/${ACADOS_GENERATED_CODE_DIRS}")
target_include_directories(nav2_dilated_polygon_nominal_controller_plugin PRIVATE ${sipoc_mr_utils_INCLUDE_DIRS})
target_include_directories(nav2_dilated_polygon_nominal_controller_plugin PRIVATE ${sipoc_mr_solver_INCLUDE_DIRS})
target_link_libraries(nav2_dilated_polygon_nominal_controller_plugin sipoc_mr_solver::dilated_polygon_nominal_controller)

ament_target_dependencies(nav2_dilated_polygon_nominal_controller_plugin  ${dependencies})


#########################################
# acados robust dilated polygon controller plugin #
#########################################

add_library(nav2_dilated_polygon_robust_controller_plugin SHARED
        src/nav2_dilated_polygon_robust_controller_plugin.cpp)

# prevent pluginlib from using boost
target_compile_definitions(nav2_dilated_polygon_robust_controller_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(nav2_dilated_polygon_robust_controller_plugin PRIVATE POLYGON_FLAG_VALUE=1)

target_include_directories(nav2_dilated_polygon_robust_controller_plugin PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(nav2_dilated_polygon_robust_controller_plugin PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(nav2_dilated_polygon_robust_controller_plugin PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(nav2_dilated_polygon_robust_controller_plugin PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(nav2_dilated_polygon_robust_controller_plugin PRIVATE "${sipoc_mr_solver_INCLUDE_DIRS}/${ACADOS_GENERATED_CODE_DIRS}")
target_include_directories(nav2_dilated_polygon_robust_controller_plugin PRIVATE ${sipoc_mr_utils_INCLUDE_DIRS})
target_include_directories(nav2_dilated_polygon_robust_controller_plugin PRIVATE ${sipoc_mr_solver_INCLUDE_DIRS})
target_link_libraries(nav2_dilated_polygon_robust_controller_plugin sipoc_mr_solver::dilated_polygon_robust_controller)

ament_target_dependencies(nav2_dilated_polygon_robust_controller_plugin  ${dependencies})

#################
# INSTALLATION  #
#################

## for correct libraries locations across platforms
include(GNUInstallDirs)

install(
  TARGETS nav2_capsule_nominal_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(nav2_capsule_nominal_controller_plugin)
pluginlib_export_plugin_description_file(nav2_core nav2_capsule_nominal_controller_plugin.xml)


install(
  TARGETS nav2_dilated_polygon_nominal_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(nav2_dilated_polygon_nominal_controller_plugin)
pluginlib_export_plugin_description_file(nav2_core nav2_dilated_polygon_nominal_controller_plugin.xml)


install(
  TARGETS nav2_dilated_polygon_robust_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(nav2_dilated_polygon_robust_controller_plugin)
pluginlib_export_plugin_description_file(nav2_core nav2_dilated_polygon_robust_controller_plugin.xml)


##########
# COMMON #
##########

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
