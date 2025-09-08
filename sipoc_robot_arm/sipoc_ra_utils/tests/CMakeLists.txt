add_executable(test_sia20d_car_seat_velocity_profile_optimizer test_sia20d_car_seat_velocity_profile_optimizer.cpp)
target_include_directories(test_sia20d_car_seat_velocity_profile_optimizer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(test_sia20d_car_seat_velocity_profile_optimizer PRIVATE
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(
  test_sia20d_car_seat_velocity_profile_optimizer
  sia20d_car_seat_velocity_profile_optimizer
  Eigen3::Eigen)
