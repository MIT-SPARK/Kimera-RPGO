ARG FROM_IMAGE=ros:eloquent

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# clone underlay source
ENV UNDERLAY_WS /opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS
COPY ./install/underlay.repos ./
RUN vcs import src < underlay.repos

# copy overlay source
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS
COPY ./ src/Kimera-RPGO
# COPY ./install/overlay.repos ./
# RUN vcs import src < overlay.repos

# copy manifests for caching
WORKDIR /opt
RUN find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp
    # && find ./ -name "COLCON_IGNORE" | \
    #   xargs cp --parents -t /tmp

# multi-stage for building
FROM $FROM_IMAGE AS build

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
      ccache \
      lcov \
      ros-$ROS_DISTRO-ros-core \
    && rm -rf /var/lib/apt/lists/*

# copy underlay manifests
ENV UNDERLAY_WS /opt/underlay_ws
COPY --from=cache /tmp/underlay_ws $UNDERLAY_WS
WORKDIR $UNDERLAY_WS

# install underlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy underlay source
COPY --from=cache $UNDERLAY_WS ./

# build underlay source
ARG UNDERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=ON \
        -DGTSAM_POSE3_EXPMAP=ON \
        -DGTSAM_ROT3_EXPMAP=ON
      # --event-handlers console_direct+

# copy overlay manifests
ENV OVERLAY_WS /opt/overlay_ws
COPY --from=cache /tmp/overlay_ws $OVERLAY_WS
WORKDIR $OVERLAY_WS

# install overlay dependencies
RUN . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
        $UNDERLAY_WS/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy overlay source
COPY --from=cache $OVERLAY_WS ./

# build overlay source
ARG OVERLAY_MIXINS="release"
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_CXX_FLAGS="\
          -Wno-parentheses \
          -Wno-reorder \
          -Wno-sign-compare \
          -Wno-unused-but-set-variable \
          -Wno-unused-function \
          -Wno-unused-parameter \
          -Wno-unused-value \
          -Wno-unused-variable"
      # --event-handlers console_direct+

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$UNDERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh