FROM osrf/ros:humble-desktop-full

# Colored prompt and ROS source
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/g' ~/.bashrc

RUN apt update \
    && apt install vim python3 python3-venv -y \
    && apt-get autoremove -y -qq \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ros2_ws

RUN python3 -m venv venv \
    && . venv/bin/activate \ 
    && pip install mujoco \
    && echo "source /home/ros2_ws/venv/bin/activate" >> ~/.bashrc

COPY . /home/ros2_ws/src/quadruped_skate_mpc

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

