#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# determine our architecture
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

DEBS_FOLDER=/etc/docker/debs
OTHER_FILES_FOLDER=/etc/docker/other_files

WORKSPACE=/tmp/workspace
cd $WORKSPACE
source /opt/ros/jazzy/setup.bash

apt-get -y update

rosdep install -y -v --from-path src/

cd src
ln -s /etc/docker/repository

git config --global --add safe.directory /etc/docker/repository

cd $WORKSPACE

colcon build --cmake-args -DENABLE_COVERAGE=true

# clean CMakeFiles files that take up too much space (majority of the build folder)
# # can not do that, we need that for coverage reporting
# cd $WORKSPACE/build
# for folder in `ls -d */`; do
#   rm -rf ${folder}/CMakeFiles;
# done

# remove stuff from src so we don't build it again
# cd $WORKSPACE/src
# rm -rf ./*

# remove logs
cd $WORKSPACE
rm -rf logs
