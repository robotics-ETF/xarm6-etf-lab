ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full as ros2-base

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    curl \
    wget \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
    iputils-ping \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=roboticsetf
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG TEMP_SRC_PATH="/home/roboticsetf"
COPY --chown=${USERNAME}:${USERNAME} . ${TEMP_SRC_PATH}

FROM ros2-base AS ros2-dev-container

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
    gdb \
    bash \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-kdl-parser \
    libeigen3-dev libkdl-parser-dev libgflags-dev libgoogle-glog-dev liborocos-kdl-dev \
    libyaml-cpp-dev liburdf-dev python3-pip libfcl-dev libnanoflann-dev \
    avahi-daemon avahi-utils lsb-release software-properties-common \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install trimesh urdfpy

ENV CLANG_VERSION=18
RUN wget https://apt.llvm.org/llvm.sh && \
    sed -i "s/add-apt-repository \"${REPO_NAME}\"/add-apt-repository \"${REPO_NAME}\" -y/g" llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh $CLANG_VERSION -y && \
    apt-get install -y clang-$CLANG_VERSION clang-tidy-$CLANG_VERSION clang-format-$CLANG_VERSION \
    llvm-$CLANG_VERSION-dev libc++-$CLANG_VERSION-dev libomp-$CLANG_VERSION-dev libc++abi-$CLANG_VERSION-dev libunwind-$CLANG_VERSION-dev && \
    rm -rf /var/lib/apt/lists/*

RUN  update-alternatives --install /usr/bin/clang clang /usr/bin/clang-$CLANG_VERSION $CLANG_VERSION \
    --slave /usr/bin/clang++ clang++ /usr/bin/clang++-$CLANG_VERSION \
    --slave /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-$CLANG_VERSION \
    --slave /usr/bin/clang-format clang-format /usr/bin/clang-format-$CLANG_VERSION

RUN ln -s /usr/bin/clangd-${CLANG_VERSION} /usr/bin/clangd

RUN echo "\n# Source ROS_DISTRO setup.bash \nsource /opt/ros/${ROS_DISTRO}/setup.bash\n" >> /etc/skel/.profile
RUN echo "\n# Add ccache to PATH\nexport PATH=/usr/lib/ccache:\$PATH\n" >> /etc/skel/.profile

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME




