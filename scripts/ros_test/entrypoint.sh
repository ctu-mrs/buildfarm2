#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

REPOSITORY_NAME=$1

WORKSPACE=/tmp/workspace
COVERAGE=/etc/docker/coverage
COREDUMP=/etc/docker/coredump

echo "$0: installing dependencies using rosdep"

rosdep install -y -v --from-path $WORKSPACE/src || echo "$0: failed to install dependencies using rosdep, the build might fail"

## | ---------------- initialize the workspace ---------------- |

if [ ! -e $WORKSPACE/install ]; then

  echo "$0: workspace not initialized, initializing"

  cd $WORKSPACE

  source /opt/ros/jazzy/setup.bash
  colcon build

fi

## | -------------------- build the package ------------------- |

echo "$0: building the workspace"

cd $WORKSPACE

source $WORKSPACE/install/setup.bash

colcon build --base-paths $WORKSPACE/src/$REPOSITORY_NAME --cmake-args -DENABLE_COVERAGE=true -DENABLE_TESTS=true

source $WORKSPACE/install/setup.bash

## | --- run tests an all ros packages within the repository -- |

echo "$0: running the tests"

cd $WORKSPACE

FAILED=0

colcon test-result --delete-yes

pkgs=$(colcon list -n --base-paths $WORKSPACE/src/$REPOSITORY_NAME)

colcon test --executor sequential --ctest-args --packages-select $pkgs

colcon test-result --all --verbose || FAILED=1

echo "$0: tests finished"

exit $FAILED
