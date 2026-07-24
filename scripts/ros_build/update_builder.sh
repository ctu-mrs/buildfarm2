#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

REPO_PATH=$MY_PATH/../..

## | -------------------------- args -------------------------- |

# INPUTS
DEBS_FOLDER=$1
BASE_IMAGE=$2
DOCKER_IMAGE=$3

[ -z $RUN_LOCALLY ] && RUN_LOCALLY=false

# default for testing

[ -z $DEBS_FOLDER ] && DEBS_FOLDER=/tmp/debs
[ -z $BASE_IMAGE ] && BASE_IMAGE=ctumrs/ros_jazzy:latest
[ -z $DOCKER_IMAGE ] && DOCKER_IMAGE=jazzy_builder

## | ----------------------------  ---------------------------- |

DEBS_EXIST=$(ls $DEBS_FOLDER 2>/dev/null | grep ".deb" | wc -l)

if [ $DEBS_EXIST -eq 0 ]; then
  echo "$0: no debs were built, leaving the builder image as it is"
  exit 0
fi

$REPO_PATH/ci_scripts/helpers/wait_for_docker.sh

if ! $RUN_LOCALLY; then

  echo "$0: logging in to docker registry"

  echo $PUSH_TOKEN | docker login ghcr.io -u ctumrsbot --password-stdin

fi

docker buildx use default

echo "$0: loading cached builder docker image"

if ! $RUN_LOCALLY; then

  docker pull ghcr.io/ctu-mrs/buildfarm2:$DOCKER_IMAGE
  docker tag ghcr.io/ctu-mrs/buildfarm2:$DOCKER_IMAGE $DOCKER_IMAGE

fi

# the Dockerfile copies the debs from /tmp/debs
if [[ "$DEBS_FOLDER" != "/tmp/debs" ]]; then
  mkdir -p /tmp/debs
  cp $DEBS_FOLDER/*.deb /tmp/debs/
fi

echo "$0: updating the builder docker image"

cd $MY_PATH

PASS_TO_DOCKER_BUILD="Dockerfile /tmp/debs"

tar -czh $PASS_TO_DOCKER_BUILD 2>/dev/null | docker build - --target squash_builder --file Dockerfile --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg BUILDER_IMAGE=${DOCKER_IMAGE} --tag ${DOCKER_IMAGE} --progress plain

echo "$0: exporting the builder docker image as ${DOCKER_IMAGE}"

if ! $RUN_LOCALLY; then

  docker tag $DOCKER_IMAGE ghcr.io/ctu-mrs/buildfarm2:$DOCKER_IMAGE
  docker push ghcr.io/ctu-mrs/buildfarm2:$DOCKER_IMAGE

fi
