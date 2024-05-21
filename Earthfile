VERSION 0.8
# Automated build of CI/CD for the project

# HOWTO run earthly CI locally
#=============================
## Test the compilation of the package with the CI:
### earthly --artifact +build-image/* ./ws_ros2_artifacts/
## Test the format checks of the package with the CI:
### earthly +check-format

# The docker image the build will run in
FROM ubuntu:22.04
WORKDIR /earthly_workdir

ARG --global python = python3.10
ARG --global ROS_DOCKER_TAG = humble-ros-base-jammy

ci-cd-sources:
    RUN mkdir -p .ci_cd
    COPY --dir .ci_cd/* ./.ci_cd
    SAVE ARTIFACT ./.ci_cd ./.ci_cd

format-files:
    COPY .pre-commit-config.yaml ./
    COPY .codespell-ignore-words.txt ./
    COPY .clang-format ./
    COPY --dir .vscode ./
    SAVE ARTIFACT ./*

git-deps:
    RUN apt-get update -y -qq
    RUN apt-get install -y -qq git

python-deps:
    FROM +git-deps
    RUN ln -s /usr/share/zoneinfo/GMT /etc/localtime
    RUN echo $CONTAINER_TIMEZONE > /etc/timezone
    RUN apt-get update -y -qq
    RUN apt-get install -y -qq $python python3-pip
    RUN $python -m pip install --upgrade pip

###################
## Documentation ##
###################

top-doc-sources:
    COPY README.md LICENSE ./
    SAVE ARTIFACT ./*

####################################
## secured_1d_velocity_controller ##
####################################

s1d-vel-dir:
    RUN mkdir -p src/secured_1d_velocity_controller
    SAVE ARTIFACT src/secured_1d_velocity_controller src/secured_1d_velocity_controller

s1d-vel-sources:
    FROM +s1d-vel-dir
    COPY --dir (./secured_1d_velocity_controller/+sources/*)  src/secured_1d_velocity_controller/
    SAVE ARTIFACT src/secured_1d_velocity_controller/* src/secured_1d_velocity_controller/

s1d-vel-test-sources:
    FROM +s1d-vel-dir
    # Get the tests of the project
    COPY --dir (./secured_1d_velocity_controller/+test-sources/*) src/secured_1d_velocity_controller/
    SAVE ARTIFACT src/secured_1d_velocity_controller/* src/secured_1d_velocity_controller/

###################################
## secured_1d_control_interfaces ##
###################################

s1d-iface-dir:
    RUN mkdir -p src/secured_1d_control_interfaces
    SAVE ARTIFACT src/secured_1d_control_interfaces src/secured_1d_control_interfaces

s1d-iface-sources:
    FROM +s1d-iface-dir
    COPY --dir (./secured_1d_control_interfaces/+sources/*) src/secured_1d_control_interfaces/
    SAVE ARTIFACT src/secured_1d_control_interfaces/* src/secured_1d_control_interfaces/

###############
## ROS image ##
###############

build-image:
    FROM ros:$ROS_DOCKER_TAG
    RUN mkdir -p ws_ros2/src/secured_1d_velocity_controller
    RUN mkdir -p ws_ros2/src/secured_1d_control_interfaces
    # Get the code of the project to document
    COPY --dir (+top-doc-sources/*) ws_ros2/src/secured_1d_velocity_controller
    COPY --dir (+s1d-vel-sources/src/secured_1d_velocity_controller/*) ws_ros2/src/secured_1d_velocity_controller
    COPY --dir (+s1d-vel-test-sources/src/secured_1d_velocity_controller/*) ws_ros2/src/secured_1d_velocity_controller
    COPY --dir (+s1d-iface-sources/src/secured_1d_control_interfaces/*) ws_ros2/src/secured_1d_control_interfaces

    LET UPDATE_CMD = "apt-get update -y -qq"
    RUN /bin/bash -c "$UPDATE_CMD"

    #########################
    # Extra-ros2 dependencies
    #########################

    # WARNING
    # =======
    # Install extra-ros2 dependencies before ros2 workspace setup
    # For instance python3-pip install failed (pkg is not found) if the
    #  installation is done after the setup of the ros2 workspace

    # git dependencies
    RUN apt-get install -y -qq git

    # python dependencies
    RUN apt-get install -y -qq $python python3-pip
    RUN $python -m pip install --upgrade pip

    ##########################
    # Setup the ros2 workspace
    ##########################
    LET ROS2_WS_SETUP_CMD = "cd ws_ros2/ && " \
        "source /opt/ros/$ROS_DISTRO/setup.bash && " \
        "rosdep install --ignore-src --from-paths . -y -r && " \
        "colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && " \
        "rm -rf /var/lib/apt/lists/* "
    RUN /bin/bash -c "$ROS2_WS_SETUP_CMD"

    # Uncomment the following line beginning with 'ENTRYPOINT' TO DEBUG THE BUILD
    # 0. build the image: `earthly +build-image`
    # 1. start the container: `docker run --name debug_build_ros2 ros2_for_sec1d_ctrl:latest`
    # 2. enter the container: `docker exec -it debug_build_ros2 /bin/bash`
    ENTRYPOINT ["tail", "-f", "/dev/null"]
    SAVE ARTIFACT ws_ros2 ws_ros2
    SAVE IMAGE ros2_for_sec1d_ctrl:latest

#######################
## Code format tests ##
#######################

check-format-deps:
    FROM +python-deps
    # Install the dependencies for the code formatter
    RUN apt-get update -y -qq
    RUN apt-get install -qq -y clang-format-14 cppcheck python3-autopep8 ament-cmake-uncrustify python3-ament-cpplint python3-ament-lint-cmake python3-ament-copyright
    RUN $python -m pip install doc8
    RUN $python -m pip install pre-commit

check-format:
    FROM +check-format-deps
    ARG GIT_REPO_URL
    ARG HTTPS=true
    LET prefix="https"
    IF [ $HTTPS = "false" ]
        SET prefix="http"
    END
    # RUN --no-cache echo "Display the URL: $repo_url"
    RUN --secret TOKEN=GIT_TOKEN git clone $prefix"://x-access-token:"$TOKEN"@"$GIT_REPO_URL repo
    # check that the code is formatted correctly
    RUN cd repo && \
        pre-commit run --show-diff-on-failure --color=always --hook-stage manual --all-files


#For debugging
docker-debug:
    FROM ros:$ROS_DOCKER_TAG
    COPY --dir ( +s1d-vel-sources/* ) .
    COPY --dir ( +top-doc-sources/* ) .
    ENTRYPOINT ["tail", "-f", "/dev/null"]
    SAVE IMAGE debug_sec1d_ctrl:latest
