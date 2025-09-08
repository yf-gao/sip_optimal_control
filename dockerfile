ARG ROS2_VERSION=humble
FROM ros:${ROS2_VERSION}

LABEL maintainer="Yunfan Gao <rubygaoyunfan@gmail.com>"
LABEL description="Dockerfile for building and running SIPOC."
LABEL version="0.0"

SHELL ["/bin/bash", "-c"]

# General preparations
RUN apt-get update \
    && apt-get install -y \
        python3-pip \
        wget \
        vim \
        rustc \
        cargo \
        libxcb-cursor-dev \
    && rm -rf /var/lib/apt/lists/*
RUN cargo install --force cbindgen
ENV PATH="$PATH:/root/.cargo/bin"

# Update rosdep indexes
RUN rosdep update

# Create the workspace
RUN mkdir -p _ws/src/sipoc

WORKDIR /_ws/src/sipoc

COPY ./ ./

WORKDIR /_ws

# Pull third-party dependencies from .repos file
RUN vcs import src < src/sipoc/third_party/third_party.repos

# Install all dependencies
RUN apt-get update -qq \
    && rosdep install --from-paths src --ignore-src -r -y --skip-keys 'OpenVDB pybind11-dev Clarabel' \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --upgrade scipy && pip install --upgrade pybind11 && pip install alphashape pyqt5
# alphashape and pyqt5 are installed only for plotting and visualization

# Build and install acados
ARG ACADOS_SOURCE_DIR=/_ws/src/third_party/acados
WORKDIR ${ACADOS_SOURCE_DIR}
RUN git submodule update --recursive --init && mkdir build
WORKDIR ${ACADOS_SOURCE_DIR}/build
RUN cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DACADOS_INSTALL_DIR="/usr" \
    && make install -j4
RUN pip install -e ${ACADOS_SOURCE_DIR}/interfaces/acados_template \
    && touch ${ACADOS_SOURCE_DIR}/COLCON_IGNORE

WORKDIR /_ws/src/third_party/acados/bin
RUN wget -O t_renderer 'https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux' \
    && chmod +x t_renderer

# Build manif and OpenVDB
WORKDIR /_ws
RUN export MAKEFLAGS="-j1" && colcon build --packages-select manif OpenVDB --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build Clarabel
WORKDIR /_ws/src/third_party/Clarabel.cpp
RUN git submodule update --init --recursive && mkdir cmake
WORKDIR /_ws/src
RUN cat ./sipoc/third_party/cmake/Clarabel_CMakeLists_addon.txt >> ./third_party/Clarabel.cpp/CMakeLists.txt \
    && cp ./sipoc/third_party/cmake/Clarabel.pc ./third_party/Clarabel.cpp/cmake/
WORKDIR /_ws
RUN colcon build --packages-select Clarabel --cmake-args -DCMAKE_BUILD_TYPE=Release -DCLARABEL_CARGO_HOME=./build/Clarabel

# Build coal
RUN export MAKEFLAGS="-j1" && colcon build --packages-up-to coal --cmake-args -DCMAKE_BUILD_TYPE=Release -DCOAL_USE_FLOAT_PRECISION=ON -DCOAL_HAS_QHULL=ON -DINSTALL_DOCUMENTATION=OFF -DBUILD_PYTHON_INTERFACE=OFF

# Generate acados code
WORKDIR /_ws
RUN colcon build --packages-select sipoc_mr_support
# The following commands will run into error because the executable cannot be built because it cannot find the blasfeo library. It does not matter because we do not need the executable.
RUN source ./install/setup.bash && python3 ./src/sipoc/sipoc_mobile_robot/sipoc_mr_solver/py_scripts/generate_code_for_acados_solver.py --numerical_eval || true
RUN python3 ./src/sipoc/sipoc_robot_arm/sipoc_ra_solver/py_scripts/generate_code_car_seat_min_length_ocp_solver.py || true
RUN python3 ./src/sipoc/sipoc_robot_arm/sipoc_ra_solver/py_scripts/generate_code_car_seat_trajectory_tracking_ocp_solver.py || true
RUN mkdir -p ./src/sipoc/sipoc_mobile_robot/sipoc_mr_solver/include/acados_generated_code/ \
    && cp -r ./c_generated_code_sipoc_mr/ ./src/sipoc/sipoc_mobile_robot/sipoc_mr_solver/include/acados_generated_code/
RUN mkdir -p ./src/sipoc/sipoc_robot_arm/sipoc_ra_solver/include/acados_generated_code/ \
    && cp -r ./c_generated_code_sipoc_ra_planning/ ./src/sipoc/sipoc_robot_arm/sipoc_ra_solver/include/acados_generated_code/ \
    && cp -r ./c_generated_code_sipoc_ra_mpc/ ./src/sipoc/sipoc_robot_arm/sipoc_ra_solver/include/acados_generated_code/ \
    && rm acados_*.json && rm -r ./c_generated_code_*

# Build mobile robot packages
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" \
    && colcon build --packages-select sipoc_plot_utils sipoc_ros2_interfaces sipoc_mr_utils sipoc_mr_solver sipoc_mr_nav2_controller --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PYBIND_LIB=ON -DBUILD_TESTING=ON -DACADOS_INSTALL_DIR=/usr
# Build robot arm packages
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" \
    && colcon build --packages-select sipoc_ra_utils sipoc_ra_support  sipoc_ra_solver sipoc_ra_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_PYBIND_LIB=ON -DOpenVDB_DIR=$PWD/install/OpenVDB/lib/cmake/OpenVDB -DACADOS_INSTALL_DIR=/usr \
    && rm -rf log

# Entry point
WORKDIR /
COPY ./.docker/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
