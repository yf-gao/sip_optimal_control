cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

project(sipoc_mr_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_TESTING ON)
option(BUILD_PYBIND_LIB "Build Pybind lib" ON)
option(CONTROLLER_DEBUG_PRINTING OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/Modules)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(manif REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED)
find_package(CLARABEL REQUIRED)
find_package(GSL REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)

include_directories(
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(DEFINED ACADOS_INSTALL_DIR)
    set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
else()
    set(ACADOS_INCLUDE_DIR "$ENV{ACADOS_SOURCE_DIR}/include")
endif()

add_library(ellip_unc_prop SHARED src/ellipsoidal_uncertainty_propagation.cpp)
target_include_directories(ellip_unc_prop PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(ellip_unc_prop PUBLIC "${ACADOS_INCLUDE_DIR}/")
target_include_directories(ellip_unc_prop PUBLIC "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
target_include_directories(ellip_unc_prop PUBLIC "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_link_libraries(ellip_unc_prop PUBLIC Eigen3::Eigen)

add_library(signed_distance_map SHARED src/signed_distance_map.cpp)
target_compile_definitions(signed_distance_map PRIVATE POLYGON_FLAG_VALUE=0)
target_include_directories(signed_distance_map PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(signed_distance_map PUBLIC ${manif_INCLUDE_DIRS})
target_link_libraries(signed_distance_map PUBLIC fmt::fmt)
target_link_libraries(signed_distance_map PUBLIC Eigen3::Eigen MANIF::manif)

add_library(signed_distance_map_for_polygon SHARED src/signed_distance_map.cpp)
target_compile_definitions(signed_distance_map_for_polygon PRIVATE POLYGON_FLAG_VALUE=1)
target_include_directories(signed_distance_map_for_polygon PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(signed_distance_map_for_polygon PUBLIC ${manif_INCLUDE_DIRS})
target_link_libraries(signed_distance_map_for_polygon PUBLIC fmt::fmt)
target_link_libraries(signed_distance_map_for_polygon PUBLIC Eigen3::Eigen MANIF::manif)

add_library(distance_point_polygon SHARED src/distance_minimizer_point_to_polygon.cpp)
if (CONTROLLER_DEBUG_PRINTING)
    target_compile_definitions(distance_point_polygon PUBLIC DEBUG_PRINTING)
endif()
target_include_directories(distance_point_polygon PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(distance_point_polygon PUBLIC ${CLARABEL_INCLUDE_DIRS})
target_link_libraries(distance_point_polygon PUBLIC Eigen3::Eigen)
target_link_libraries(distance_point_polygon PUBLIC CLARABEL::clarabel)

add_library(distance_ellip_polygon SHARED src/distance_minimizer_ellipsoid_to_polygon.cpp)
if (CONTROLLER_DEBUG_PRINTING)
    target_compile_definitions(distance_ellip_polygon PUBLIC DEBUG_PRINTING)
endif()
target_include_directories(distance_ellip_polygon PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(distance_ellip_polygon PUBLIC ${CLARABEL_INCLUDE_DIRS})
target_link_libraries(distance_ellip_polygon PUBLIC Eigen3::Eigen)
target_link_libraries(distance_ellip_polygon PUBLIC CLARABEL::clarabel)

add_library(state_predictor_filter SHARED src/state_predictor_filter.cpp)
target_include_directories(state_predictor_filter PUBLIC ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
target_link_libraries(state_predictor_filter PUBLIC Eigen3::Eigen)

add_library(vel_profile_optimizer SHARED src/spline_fitter.cpp src/spline_wrapper.cpp src/time_optimal_velocity.cpp src/velocity_profile_optimizer.cpp)
target_link_libraries(vel_profile_optimizer PUBLIC Eigen3::Eigen MANIF::manif GSL::gsl)

if(BUILD_PYBIND_LIB)
  pybind11_add_module(py_sd_map src/pybind/signed_distance_map_pybind.cpp)
  target_include_directories(py_sd_map PUBLIC ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(py_sd_map PUBLIC ${PYTHON_LIBRARIES} pybind11::module)

  pybind11_add_module(py_distance_point_polygon SHARED src/pybind/distance_minimizer_point_to_polygon_pybind.cpp)
  target_include_directories(py_distance_point_polygon PUBLIC ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(py_distance_point_polygon PUBLIC distance_point_polygon)
  target_link_libraries(py_distance_point_polygon PUBLIC CLARABEL::clarabel)
  target_link_libraries(py_distance_point_polygon PUBLIC ${PYTHON_LIBRARIES} pybind11::module)

  pybind11_add_module(py_distance_ellip_polygon SHARED src/pybind/distance_minimizer_ellipsoid_to_polygon_pybind.cpp)
  target_include_directories(py_distance_ellip_polygon PUBLIC ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(py_distance_ellip_polygon PUBLIC distance_ellip_polygon)
  target_link_libraries(py_distance_ellip_polygon PUBLIC CLARABEL::clarabel)
  target_link_libraries(py_distance_ellip_polygon PUBLIC ${PYTHON_LIBRARIES} pybind11::module)

  pybind11_add_module(py_vel_profile_optimizer src/pybind/velocity_profile_optimizer_pybind.cpp)
  target_link_libraries(py_vel_profile_optimizer PUBLIC vel_profile_optimizer)
  target_link_libraries(py_vel_profile_optimizer PUBLIC ${Python3_LIBRARIES} pybind11::module)

endif(BUILD_PYBIND_LIB)

# Add test subfolder
if(BUILD_TESTING)
  add_subdirectory(tests)
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
  TARGETS signed_distance_map signed_distance_map_for_polygon ellip_unc_prop distance_point_polygon distance_ellip_polygon state_predictor_filter vel_profile_optimizer
  EXPORT sipoc_mr_utils_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
ament_export_targets(sipoc_mr_utils_targets HAS_LIBRARY_TARGET)


if (NOT CMAKE_INSTALL_PYTHON_LIBDIR)
  set(CMAKE_INSTALL_PYTHON_LIBDIR
    "${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif ()
if(BUILD_PYBIND_LIB)
  install(
      TARGETS py_sd_map py_distance_point_polygon py_distance_ellip_polygon py_vel_profile_optimizer
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_PYTHON_LIBDIR}/py_sipoc_mr_utils
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  install(FILES src/pybind/__init__.py
    DESTINATION ${CMAKE_INSTALL_PYTHON_LIBDIR}/py_sipoc_mr_utils
  )
  # Add custom target to install Python package
  add_custom_target(install_python_package
    COMMAND ${Python3_EXECUTABLE} -m pip install .
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  # Ensure the custom target is run after the build
  add_dependencies(py_sd_map install_python_package)
  add_dependencies(py_distance_point_polygon install_python_package)
  add_dependencies(py_distance_ellip_polygon install_python_package)
  add_dependencies(py_vel_profile_optimizer install_python_package)
endif(BUILD_PYBIND_LIB)

ament_package()
