FROM nvidia-ros-noetic:latest
SHELL [ "/bin/bash", "-c" ]

ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=noetic

RUN useradd --create-home appuser

USER appuser
WORKDIR /appuser
RUN mkdir -p catkin_ws/src
COPY ./src catkin_ws/src

USER root
RUN apt-get update --fix-missing \
  && rosdep update
WORKDIR /appuser/catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y

USER appuser
WORKDIR /home/appuser
RUN git clone https://github.com/osrf/gazebo_models.git
RUN mkdir -p .gazebo/models \
  && cp -r gazebo_models/* .gazebo/models

USER appuser
WORKDIR /appuser/catkin_ws
RUN source /opt/ros/noetic/setup.bash \
  && catkin config --install \
  && catkin build

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /appuser/catkin_ws/devel/setup.bash" >> ~/.bashrc
