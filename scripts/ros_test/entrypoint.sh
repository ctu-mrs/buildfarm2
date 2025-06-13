#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

REPOSITORY_NAME=$1

WORKSPACE=/tmp/workspace
COVERAGE=/etc/docker/coverage
COREDUMP=/etc/docker/coredump

echo "$0: installing dependencies using rosdep"

rosdep install -y -v --from-path $WORKSPACE/src

## | ---------------- initialize the workspace ---------------- |

if [ ! -e $WORKSPACE/install ]; then

  echo "$0: workspace not initialized, initializing"

  cd $WORKSPACE

  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install

fi

## | -------------------- build the package ------------------- |

echo "$0: building the workspace"

cd $WORKSPACE

source $WORKSPACE/install/setup.bash

colcon build --cmake-args -DENABLE_COVERAGE=true -DENABLE_TESTS=true --paths $WORKSPACE/src/$REPOSITORY_NAME

## | --- run tests an all ros packages within the repository -- |

cd $WORKSPACE

FAILED=0

colcon test-result --delete-yes

colcon test --paths $WORKSPACE/src/$REPOSITORY_NAME --executor sequential --ctest-args

colcon test-result --all --verbose || FAILED=1

echo "$0: tests finished"

exit $FAILED
