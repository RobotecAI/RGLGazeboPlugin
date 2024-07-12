ARG BASE_IMAGE=base
FROM ros:jazzy AS base

# Edit apt config for caching and update once
RUN mv /etc/apt/apt.conf.d/docker-clean /etc/apt/ && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/keep-cache && \
    apt-get update

################################################################################
# MARK: prepper - prep rgl dependencies
################################################################################
### Core dependencies stage
FROM $BASE_IMAGE AS prepper

# Set working directory using standard opt path
WORKDIR /opt/rgl

# Copy package manifest
COPY package.xml src/RGLGazeboPlugin/package.xml

# Install bootstrap tools for install scripts
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    rosdep install -y \
        --from-paths src \
        --ignore-src

################################################################################
# MARK: builder - build rgl binaries
################################################################################
FROM prepper AS builder

# Copy source tree
COPY . /tmp/RGLGazeboPlugin
RUN --mount=type=cache,target=src/,rw \
    cp -r /tmp/RGLGazeboPlugin src/RGLGazeboPlugin

RUN --mount=type=cache,target=src/,rw \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

################################################################################
# MARK: dancer - multi-stage for cache dancing
################################################################################
FROM builder AS dancer

# Copy entire build directory
# RUN mkdir /dancer && \
#     cp -rT build /dancer

# Copy only the lib and bin directories
RUN --mount=type=cache,target=src/,rw \
    mkdir /dancer && \
    find install -type f -name "*.so" -exec cp {} /dancer/ \;

################################################################################
# MARK: exporter - export rgl binaries and executables
################################################################################
FROM scratch AS exporter

COPY --from=dancer /dancer /
