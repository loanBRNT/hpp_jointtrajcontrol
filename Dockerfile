# Start with an official ROS 2 base image for the desired distribution
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update

RUN apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /etc/apt/keyrings
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list

RUN sudo apt-get update

RUN apt-get install -y \ 
    robotpkg-py310-hpp-manipulation-corba \
    robotpkg-py310-qt5-hpp-gepetto-viewer \
    robotpkg-py310-hpp-tutorial \
    robotpkg-py310-qt5-hpp-gui \
    robotpkg-py310-qt5-hpp-plot \
    robotpkg-py310-hpp-environments \
    robotpkg-romeo-description

ENV PATH=/opt/openrobots/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
ENV ROS_PACKAGE_PATH=/opt/openrobots/share:$ROS_PACKAGE_PATH
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
ENV PKG_CONFIG_PATH=/opt/openrobots:$PKG_CONFIG_PATH

RUN sudo apt-get install -y ros-humble-franka-description
RUN sudo apt update && apt install -y ros-humble-control-msgs ros-humble-trajectory-msgs ros-humble-rmw-cyclonedds-cpp

ENV ROS_PACKAGE_PATH=/opt/ros/humble/share:$ROS_PACKAGE_PATH

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT []
WORKDIR /ros2_ws