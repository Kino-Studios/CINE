# Gazebo Harmonic + ArduPilot SITL + ROS 2 Jazzy image for simulating the CINE drone stack.
# Every source is pinned: base image digest, dated apt snapshots, git commits, exact Python
# versions, and (in fetch_weights.py) model revisions and file hashes.
#
# Build and run from the CINE repository root:
#   docker compose -f docker/gazebo-sim/compose.yaml build
#   docker compose -f docker/gazebo-sim/compose.yaml up

FROM ubuntu:noble-20251013@sha256:c35e29c9450151419d9448b0fd75374fec4fff364a27f176fb458d472dfc9e54

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

ARG UBUNTU_SNAPSHOT=https://snapshot.ubuntu.com/ubuntu/20261114T000000Z
ARG ROS_SNAPSHOT=http://snapshots.ros.org/jazzy/2026-10-05/ubuntu
# ASSUMPTION: fingerprint of the key that signs snapshots.ros.org (the "ROS snapshots"
# key the official EOL ROS images use), which differs from the packages.ros.org key.
# gpg rejects a key served under any other fingerprint, so a wrong value fails the build
# here rather than installing unsigned packages.
ARG ROS_SNAPSHOT_KEY=4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# The base image lacks ca-certificates, so the first update over HTTPS skips TLS peer
# checks; apt still verifies every index against ubuntu-keyring, which the base image
# ships, so package integrity does not rest on TLS.
RUN set -eux; \
    sed -i -E "s#^URIs: http://(archive|security)\.ubuntu\.com/ubuntu/?\$#URIs: ${UBUNTU_SNAPSHOT}#" \
        /etc/apt/sources.list.d/ubuntu.sources; \
    if grep '^URIs:' /etc/apt/sources.list.d/ubuntu.sources | grep -qv 'snapshot\.ubuntu\.com'; then \
        echo 'ubuntu.sources still names an archive other than the snapshot' >&2; \
        exit 1; \
    fi; \
    # A dated snapshot keeps serving the same Release files, so their expiry must not
    # block a rebuild months later.
    echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99-snapshot-valid-until; \
    apt-get -o Acquire::https::Verify-Peer=false -o Acquire::https::Verify-Host=false update; \
    apt-get -o Acquire::https::Verify-Peer=false -o Acquire::https::Verify-Host=false \
        install -y --no-install-recommends ca-certificates gnupg; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver hkps://keyserver.ubuntu.com --recv-keys "${ROS_SNAPSHOT_KEY}"; \
    gpg --batch --export "${ROS_SNAPSHOT_KEY}" > /usr/share/keyrings/ros-snapshots-archive-keyring.gpg; \
    test -s /usr/share/keyrings/ros-snapshots-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "${GNUPGHOME}"; \
    echo "deb [signed-by=/usr/share/keyrings/ros-snapshots-archive-keyring.gpg] ${ROS_SNAPSHOT} noble main" \
        > /etc/apt/sources.list.d/ros2-snapshots.list; \
    rm -rf /var/lib/apt/lists/*

# Beyond the simulator itself, this installs what colcon needs to build the three mounted
# repositories (CINE with ego_planner_swarm, CINE-Sensing with VINS-Fusion, CINE-Interfaces),
# because the build runs in the entrypoint where rosdep is not available. sudo, lsb-release
# and tzdata are what ArduPilot's install-prereqs-ubuntu.sh expects to find already.
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-sim-vendor \
        ros-jazzy-mavros \
        ros-jazzy-mavros-extras \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-pcl-conversions \
        ros-jazzy-pcl-ros \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-laser-geometry \
        ros-jazzy-rviz2 \
        python3-colcon-common-extensions \
        python3-venv \
        python3-numpy \
        python3-opencv \
        python3-yaml \
        git \
        wget \
        build-essential \
        cmake \
        sudo \
        lsb-release \
        tzdata \
        rapidjson-dev \
        libopencv-dev \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-libav \
        gstreamer1.0-gl \
        geographiclib-tools \
        libeigen3-dev \
        libboost-all-dev \
        libpcl-dev \
        libarmadillo-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libatlas-base-dev \
        libsuitesparse-dev; \
    rm -rf /var/lib/apt/lists/*

# MAVROS converts between geoid and ellipsoid heights with these datasets. The script
# downloads them from GeographicLib's own server, which has no pinned version.
RUN set -eux; \
    datasets_script="$(find /opt/ros -name 'install_geographiclib_datasets.sh' -print -quit)"; \
    if [ -z "${datasets_script}" ]; then \
        echo 'install_geographiclib_datasets.sh not found under /opt/ros' >&2; \
        exit 1; \
    fi; \
    bash "${datasets_script}"

# VINS-Fusion uses ceres::LocalParameterization, which Ceres 2.2 (Ubuntu 24.04's package)
# removed. Same source, checksum and flags as CINE-Sensing's tools/install_ceres_2_1.sh.
ARG CERES_VERSION=2.1.0
ARG CERES_SHA256=ccbd716a93f65d4cb017e3090ae78809e02f5426dce16d0ee2b4f8a4ba2411a8
RUN set -eux; \
    work_dir="$(mktemp -d)"; \
    archive="${work_dir}/ceres-solver-${CERES_VERSION}.tar.gz"; \
    wget -q "https://github.com/ceres-solver/ceres-solver/archive/refs/tags/${CERES_VERSION}.tar.gz" -O "${archive}"; \
    echo "${CERES_SHA256}  ${archive}" | sha256sum -c -; \
    tar xzf "${archive}" -C "${work_dir}"; \
    # Ubuntu 24.04's SuiteSparse no longer provides CXSparse.
    cmake -S "${work_dir}/ceres-solver-${CERES_VERSION}" -B "${work_dir}/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DCXSPARSE=OFF; \
    cmake --build "${work_dir}/build" -j "$(nproc)"; \
    cmake --install "${work_dir}/build"; \
    ldconfig; \
    rm -rf "${work_dir}"

# install-prereqs-ubuntu.sh refuses to run as root and calls sudo itself. sudo resets the
# environment, so DEBIAN_FRONTEND is kept explicitly to stop tzdata and friends prompting.
RUN set -eux; \
    useradd --create-home --shell /bin/bash ardupilot; \
    echo 'ardupilot ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ardupilot; \
    echo 'Defaults env_keep += "DEBIAN_FRONTEND"' >> /etc/sudoers.d/ardupilot; \
    chmod 0440 /etc/sudoers.d/ardupilot; \
    install -d -o ardupilot -g ardupilot /opt/ardupilot

ARG ARDUPILOT_COMMIT=fb773567896014da007bca58358704d4a0d8516f
USER ardupilot
WORKDIR /opt/ardupilot
RUN set -eux; \
    git init --quiet .; \
    git remote add origin https://github.com/ArduPilot/ardupilot.git; \
    git fetch --depth 1 origin "${ARDUPILOT_COMMIT}"; \
    git checkout --quiet --detach FETCH_HEAD; \
    test "$(git rev-parse HEAD)" = "${ARDUPILOT_COMMIT}"; \
    git submodule update --init --recursive

# USER does not set $USER, which the script passes to usermod. DO_AP_STM_ENV=0 skips the
# ARM firmware toolchain download, which SITL does not use.
RUN set -eux; \
    USER=ardupilot HOME=/home/ardupilot \
    SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1 DO_AP_STM_ENV=0 \
        Tools/environment_install/install-prereqs-ubuntu.sh -y; \
    # ASSUMPTION: on Ubuntu 24.04 the script puts its Python packages (empy, pymavlink,
    # MAVProxy) in this venv; waf below needs them.
    if [ ! -x /home/ardupilot/venv-ardupilot/bin/python ]; then \
        echo 'install-prereqs-ubuntu.sh did not create /home/ardupilot/venv-ardupilot' >&2; \
        exit 1; \
    fi

# The script activates its venv from ~/.bashrc, which RUN never reads, so PATH is set here.
RUN set -eux; \
    export PATH="/home/ardupilot/venv-ardupilot/bin:${PATH}"; \
    ./waf configure --board sitl; \
    ./waf copter; \
    test -x /opt/ardupilot/build/sitl/bin/arducopter

USER root
WORKDIR /

ENV GZ_VERSION=harmonic
ARG ARDUPILOT_GAZEBO_COMMIT=f116b03552d05c8d0b0bed5ed67033a1183121d1
# Sourcing ROS puts the ROS-vendored Gazebo libraries (gz_sim_vendor) on CMAKE_PREFIX_PATH.
RUN set -eux; \
    install -d /opt/ardupilot_gazebo; \
    cd /opt/ardupilot_gazebo; \
    git init --quiet .; \
    git remote add origin https://github.com/ArduPilot/ardupilot_gazebo.git; \
    git fetch --depth 1 origin "${ARDUPILOT_GAZEBO_COMMIT}"; \
    git checkout --quiet --detach FETCH_HEAD; \
    test "$(git rev-parse HEAD)" = "${ARDUPILOT_GAZEBO_COMMIT}"; \
    set +u; \
    source /opt/ros/jazzy/setup.bash; \
    set -u; \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo; \
    cmake --build build -j "$(nproc)"; \
    test -f build/libArduPilotPlugin.so

ENV ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds \
    HF_HOME=/weights/huggingface

# --system-site-packages lets the venv see apt's numpy 1.26.4, which cv_bridge is built
# against; pinning numpy==1.26.4 here stops any package below pulling numpy 2. scipy is
# pinned to the newest release that still accepts that numpy; cine_eval's motion channel
# imports it, and the scoring step runs in this venv. Depth
# Anything 3 goes in with --no-deps because its declared dependencies add xformers, open3d,
# COLMAP bindings and a web server that cine_depth never imports.
RUN set -eux; \
    python3 -m venv --system-site-packages /opt/cine-venv; \
    /opt/cine-venv/bin/pip install --no-cache-dir \
        numpy==1.26.4 \
        scipy==1.16.3 \
        torch==2.15.0 \
        torchvision==0.29.2 \
        huggingface_hub==2.4.0 \
        safetensors==0.8.0 \
        einops==0.8.2 \
        imageio==2.38.1 \
        omegaconf==2.4.0 \
        addict==2.4.0 \
        evo==1.38.1 \
        ultralytics==8.4.213; \
    /opt/cine-venv/bin/pip install --no-cache-dir --no-deps \
        "depth-anything-3 @ git+https://github.com/ByteDance-Seed/Depth-Anything-3@3d835ec1a5802d64a8b8b15f817a1ab54809bfe4"

COPY fetch_weights.py /opt/cine/fetch_weights.py
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod 0755 /usr/local/bin/entrypoint.sh

RUN dpkg-query -W > /apt-versions.txt

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
