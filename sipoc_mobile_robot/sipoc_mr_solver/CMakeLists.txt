cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

project(sipoc_mr_solver)

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
find_package(manif REQUIRED)
find_package(sipoc_mr_utils REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(CLARABEL REQUIRED)
find_package(fmt REQUIRED)
find_package(GSL REQUIRED)

include_directories(
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(DEFINED ACADOS_INSTALL_DIR)
    set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
    set(ACADOS_LIB_DIR "${ACADOS_INSTALL_DIR}/lib")
else()
    set(ACADOS_INCLUDE_DIR "$ENV{ACADOS_SOURCE_DIR}/include")
    set(ACADOS_LIB_DIR "$ENV{ACADOS_SOURCE_DIR}/lib")
endif()
set(ACADOS_GENERATED_CODE_DIRS acados_generated_code/c_generated_code_sipoc_mr)

add_library(capsule_nominal_controller SHARED
    src/capsule_nominal_controller.cpp src/base_controller.cpp
    "include/${ACADOS_GENERATED_CODE_DIRS}/acados_sim_solver_diff_drive_robot.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/acados_solver_diff_drive_robot.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_ode_fun.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_vde_adj.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_vde_forw.c")
if (CONTROLLER_DEBUG_PRINTING)
    target_compile_definitions(capsule_nominal_controller PRIVATE DEBUG_PRINTING)
endif()
target_compile_definitions(capsule_nominal_controller PRIVATE POLYGON_FLAG_VALUE=0)
target_include_directories(capsule_nominal_controller PUBLIC ${sipoc_mr_utils_INCLUDE_DIRS})
target_include_directories(capsule_nominal_controller PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${ACADOS_GENERATED_CODE_DIRS}>
    $<INSTALL_INTERFACE:include/${ACADOS_GENERATED_CODE_DIRS}>
)
target_include_directories(capsule_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(capsule_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(capsule_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
target_link_libraries(capsule_nominal_controller Eigen3::Eigen MANIF::manif)
target_link_libraries(capsule_nominal_controller "${ACADOS_LIB_DIR}/libacados.so")
ament_target_dependencies(capsule_nominal_controller sipoc_mr_utils)

add_library(dilated_polygon_nominal_controller SHARED
    src/dilated_polygon_nominal_controller.cpp src/base_controller.cpp
    "include/${ACADOS_GENERATED_CODE_DIRS}/acados_sim_solver_diff_drive_robot.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/acados_solver_diff_drive_robot.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_ode_fun.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_vde_adj.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_vde_forw.c")
if (CONTROLLER_DEBUG_PRINTING)
    target_compile_definitions(dilated_polygon_nominal_controller PRIVATE DEBUG_PRINTING)
endif()
target_compile_definitions(dilated_polygon_nominal_controller PRIVATE POLYGON_FLAG_VALUE=1)
target_include_directories(dilated_polygon_nominal_controller PUBLIC ${sipoc_mr_utils_INCLUDE_DIRS})
target_include_directories(dilated_polygon_nominal_controller PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${ACADOS_GENERATED_CODE_DIRS}>
    $<INSTALL_INTERFACE:include/${ACADOS_GENERATED_CODE_DIRS}>
)
target_include_directories(dilated_polygon_nominal_controller PUBLIC ${CLARABEL_INCLUDE_DIRS})
target_include_directories(dilated_polygon_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(dilated_polygon_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(dilated_polygon_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
target_link_libraries(dilated_polygon_nominal_controller Eigen3::Eigen MANIF::manif)
target_link_libraries(dilated_polygon_nominal_controller "${ACADOS_LIB_DIR}/libacados.so")
target_link_libraries(dilated_polygon_nominal_controller CLARABEL::clarabel)
ament_target_dependencies(dilated_polygon_nominal_controller sipoc_mr_utils)

add_library(dilated_polygon_robust_controller SHARED
    src/dilated_polygon_robust_controller.cpp src/base_controller.cpp
    "include/${ACADOS_GENERATED_CODE_DIRS}/acados_sim_solver_diff_drive_robot.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/acados_solver_diff_drive_robot.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_ode_fun.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_vde_adj.c"
    "include/${ACADOS_GENERATED_CODE_DIRS}/diff_drive_robot_model/diff_drive_robot_expl_vde_forw.c")
if (CONTROLLER_DEBUG_PRINTING)
    target_compile_definitions(dilated_polygon_robust_controller PRIVATE DEBUG_PRINTING)
endif()
target_compile_definitions(dilated_polygon_robust_controller PRIVATE POLYGON_FLAG_VALUE=1)
target_include_directories(dilated_polygon_robust_controller PUBLIC ${sipoc_mr_utils_INCLUDE_DIRS})
target_include_directories(dilated_polygon_robust_controller PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${ACADOS_GENERATED_CODE_DIRS}>
    $<INSTALL_INTERFACE:include/${ACADOS_GENERATED_CODE_DIRS}>
)
target_include_directories(dilated_polygon_robust_controller PUBLIC ${CLARABEL_INCLUDE_DIRS})
target_include_directories(dilated_polygon_robust_controller PRIVATE "${ACADOS_INCLUDE_DIR}/")
target_include_directories(dilated_polygon_robust_controller PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
target_include_directories(dilated_polygon_robust_controller PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
target_link_libraries(dilated_polygon_robust_controller Eigen3::Eigen MANIF::manif)
target_link_libraries(dilated_polygon_robust_controller "${ACADOS_LIB_DIR}/libacados.so")
target_link_libraries(dilated_polygon_robust_controller CLARABEL::clarabel)
ament_target_dependencies(dilated_polygon_robust_controller sipoc_mr_utils)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if (BUILD_PYBIND_LIB)
  pybind11_add_module(py_solver_status SHARED src/pybind/solver_status_pybind.cpp)
  target_include_directories(py_solver_status PUBLIC ${sipoc_mr_utils_INCLUDE_DIRS})
  target_include_directories(py_solver_status PUBLIC ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(py_solver_status PUBLIC ${PYTHON_LIBRARIES} pybind11::module)

  pybind11_add_module(py_capsule_nominal_controller SHARED src/pybind/capsule_nominal_controller_pybind.cpp)
  target_compile_definitions(py_capsule_nominal_controller PRIVATE POLYGON_FLAG_VALUE=0)
  target_include_directories(py_capsule_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/")
  target_include_directories(py_capsule_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
  target_include_directories(py_capsule_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
  target_link_libraries(py_capsule_nominal_controller PUBLIC capsule_nominal_controller)
  target_link_libraries(py_capsule_nominal_controller PUBLIC ${PYTHON_LIBRARIES} pybind11::module)

  pybind11_add_module(py_dilated_polygon_nominal_controller SHARED src/pybind/dilated_polygon_nominal_controller_pybind.cpp)
  target_compile_definitions(py_dilated_polygon_nominal_controller PRIVATE POLYGON_FLAG_VALUE=1)
  target_include_directories(py_dilated_polygon_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/")
  target_include_directories(py_dilated_polygon_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
  target_include_directories(py_dilated_polygon_nominal_controller PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
  target_link_libraries(py_dilated_polygon_nominal_controller PUBLIC dilated_polygon_nominal_controller)
  target_link_libraries(py_dilated_polygon_nominal_controller PUBLIC ${PYTHON_LIBRARIES} pybind11::module)

  pybind11_add_module(py_dilated_polygon_robust_controller SHARED src/pybind/dilated_polygon_robust_controller_pybind.cpp)
  target_compile_definitions(py_dilated_polygon_robust_controller PRIVATE POLYGON_FLAG_VALUE=1)
  target_include_directories(py_dilated_polygon_robust_controller PRIVATE "${ACADOS_INCLUDE_DIR}/")
  target_include_directories(py_dilated_polygon_robust_controller PRIVATE "${ACADOS_INCLUDE_DIR}/hpipm/include/")
  target_include_directories(py_dilated_polygon_robust_controller PRIVATE "${ACADOS_INCLUDE_DIR}/blasfeo/include/")
  target_link_libraries(py_dilated_polygon_robust_controller PUBLIC dilated_polygon_robust_controller)
  target_link_libraries(py_dilated_polygon_robust_controller PUBLIC ${PYTHON_LIBRARIES} pybind11::module)
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
  TARGETS capsule_nominal_controller dilated_polygon_nominal_controller dilated_polygon_robust_controller
  EXPORT sipoc_mr_solver_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
ament_export_targets(sipoc_mr_solver_targets HAS_LIBRARY_TARGET)

if (NOT CMAKE_INSTALL_PYTHON_LIBDIR)
  set(CMAKE_INSTALL_PYTHON_LIBDIR
    "${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif ()
if(BUILD_PYBIND_LIB)
  install(
      TARGETS py_solver_status py_capsule_nominal_controller py_dilated_polygon_nominal_controller py_dilated_polygon_robust_controller
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_PYTHON_LIBDIR}/py_sipoc_mr_solver
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  install(FILES src/pybind/__init__.py
    DESTINATION ${CMAKE_INSTALL_PYTHON_LIBDIR}/py_sipoc_mr_solver
  )
  # Add custom target to install Python package
  add_custom_target(install_python_package
    COMMAND ${Python3_EXECUTABLE} -m pip install .
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  # Ensure the custom target is run after the build
  add_dependencies(py_solver_status install_python_package)
  add_dependencies(py_capsule_nominal_controller install_python_package)
  add_dependencies(py_dilated_polygon_nominal_controller install_python_package)
  add_dependencies(py_dilated_polygon_robust_controller install_python_package)
endif(BUILD_PYBIND_LIB)

ament_package()