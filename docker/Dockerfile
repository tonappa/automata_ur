ARG ROS_DISTRO=jazzy
ARG ROS_VERSION=2

FROM ros:${ROS_DISTRO}

# ARGs reset after FROM: declared again
ARG ROS_DISTRO
ARG ROS_VERSION
ARG MYUID=1000
ARG MYGID=1000
ARG USER=ros
ARG WORKSPACE_NAME=ros_ws

ENV DEBIAN_FRONTEND=noninteractive

# Common packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    bash-completion \
    build-essential \
    python3-pip \
    python3-dev \
    python3-rosdep \
    curl \
    wget \
    unzip \
    gnupg \
    lsb-release \
    apt-utils \
    software-properties-common \
    iputils-ping \
    vim \
    figlet \
    toilet \
    libgl1 \
    libsm6 \
    libxext6 && \
    rm -rf /var/lib/apt/lists/*

# ROS 2 build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-core \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# ROS packages
RUN apt-get update && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-runtime-monitor \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-mqtt-client \
    nlohmann-json3-dev && \
    rm -rf /var/lib/apt/lists/*

# OR-Tools C++ (Apache-2.0), used by ur_automata_scan_tsp
RUN curl -fsSL -o /tmp/ortools.tgz https://github.com/google/or-tools/releases/download/v9.15/or-tools_amd64_ubuntu-24.04_cpp_v9.15.6755.tar.gz && \
    echo "6f389320672cee00b78aacefb2bde33fef0bb988c3b2735573b9fffd1047fbda  /tmp/ortools.tgz" | sha256sum -c && \
    mkdir -p /opt/ortools && tar xzf /tmp/ortools.tgz -C /opt/ortools --strip-components=1 && rm /tmp/ortools.tgz

# User with the host UID/GID
RUN OLD_USER=$(getent passwd ${MYUID} | cut -d: -f1) \
    && OLD_GROUP=$(getent group ${MYGID} | cut -d: -f1) \
    && if [ -n "${OLD_GROUP}" ] && [ "${OLD_GROUP}" != "${USER}" ]; then \
    groupmod -n ${USER} ${OLD_GROUP}; \
    fi \
    && if [ -n "${OLD_USER}" ] && [ "${OLD_USER}" != "${USER}" ]; then \
    usermod -l ${USER} ${OLD_USER} \
    && usermod -d /home/${USER} -m ${USER}; \
    fi \
    && if ! id -u ${USER} >/dev/null 2>&1; then \
    addgroup --gid ${MYGID} ${USER} \
    && adduser --gecos "ROS User" --disabled-password --uid ${MYUID} --gid ${MYGID} ${USER}; \
    fi \
    && usermod -a -G dialout,video,sudo ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER}

USER ${USER}
ENV HOME=/home/${USER}

# The workspace is mounted at runtime and built in the container.
WORKDIR /home/${USER}/${WORKSPACE_NAME}
RUN mkdir -p ./src

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sudo chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
