FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install sudo and basic tools
RUN apt-get update && apt-get install -y sudo \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git

# Don't remove this, for rosdep to work later on.
#    && rm -rf /var/lib/apt/lists/*

# Delete the default 'ubuntu' user if it exists to avoid UID/GID conflicts
RUN if id -u 1000 >/dev/null 2>&1; then userdel -r ubuntu; fi || true \
    && if getent group 1000 >/dev/null 2>&1; then groupdel $(getent group 1000 | cut -d: -f1); fi || true

# Create the user and add to sudo group
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non-root user
USER $USERNAME

# Setup workspace and permissions
WORKDIR /ros2_ws

# First, copy just the list of repos
COPY all_mola.repos .

# Create the src directory and clone everything into it
RUN mkdir src && \
    vcs import src < all_mola.repos

# Make sure we have all submodules:
RUN cd src && \
    vcs custom --args submodule update --init --recursive

# Initialize rosdep (as the new user)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build  \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --parallel-workers 2


# Entrypoint setup
RUN echo "#!/bin/bash \n set -e \n source /opt/ros/jazzy/setup.bash \n [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash \n exec \"\$@\"" > /home/$USERNAME/entrypoint.sh && \
    chmod +x /home/$USERNAME/entrypoint.sh

ENTRYPOINT ["/home/rosuser/entrypoint.sh"]
CMD ["bash"]