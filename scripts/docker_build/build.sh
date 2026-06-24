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
LIST=$1
PPA_VARIANT=$2
REPOSITORY=$3
PATH_TO_DOCKER_FOLDER=$4
BASE_IMAGE=$5
IMAGE_NAME=$6
CUSTOM_TAG=$7

[ -z $RUN_LOCALLY ] && RUN_LOCALLY=false

# default for testing

[ -z $LIST ] && LIST=mrs
[ -z $PPA_VARIANT ] && PPA_VARIANT=testing
[ -z $REPOSITORY ] && REPOSITORY=mrs_uav_autostart
[ -z $PATH_TO_DOCKER_FOLDER ] && PATH_TO_DOCKER_FOLDER=~/git/realsense/docker
[ -z $BASE_IMAGE ] && BASE_IMAGE=ctumrs/ros_jazzy:latest
[ -z $IMAGE_NAME ] && IMAGE_NAME=$REPOSITORY
[ -z $CUSTOM_TAG ] && CUSTOM_TAG=$PPA_VARIANT

## | ---------------------- derived args ---------------------- |

# determine our architecture
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

YAML_FILE=$REPO_PATH/${LIST}.yaml

# needed for building open_vins
export ROS_VERSION=1

REPOS=$($REPO_PATH/scripts/helpers/get_repo_source_for_docker.py $YAML_FILE $PPA_VARIANT $ARCH $REPOSITORY)

PLATFORM=""

# clone and checkout
while IFS= read -r REPO; do

  cd /tmp

  sudo rm -rf repository

  REPO_NAME=$(echo "$REPO" | awk '{print $1}')
  URL=$(echo "$REPO" | awk '{print $2}')
  BRANCH=$(echo "$REPO" | awk '{print $3}')
  GITMAN=$(echo "$REPO" | awk '{print $4}')
  AMD=$(echo "$REPO" | awk '{print $5}')
  ARM=$(echo "$REPO" | awk '{print $6}')

  if [[ "$AMD" == "True" ]] && [[ "$ARM" == "True" ]]; then
    PLATFORM="linux/amd64,linux/arm64"
  elif [[ "$AMD" == "True" ]]; then
    PLATFORM="linux/amd64"
  elif [[ "$ARM" == "True" ]]; then
    PLATFORM="linux/arm64"
  fi

  echo "$0: cloning '$URL --depth 1 --branch $BRANCH' into '$REPO'"
  [ -e repository ] && rm -rf repository || git clone $URL --recurse-submodules --depth 1 --branch $BRANCH repository

  if [[ "$GITMAN" == "True" ]]; then
    cd repository
    pipx install gitman==3.5.2 --pip-args regex==2024.9.11
    [[ -e .gitman.yml || -e .gitman.yaml ]] && gitman install
  fi

done < <(echo "$REPOS")

echo "$0: repository cloned to /tmp/repository"

## --------------------------------------------------------------
## |                        docker build                        |
## --------------------------------------------------------------

$REPO_PATH/ci_scripts/helpers/wait_for_docker.sh

if ! $RUN_LOCALLY; then

  echo "$0: logging in to docker registry"

  docker login --username klaxalk --password $DOCKERHUB

fi

## | ---------------------- run the test ---------------------- |

# if there are any artifacts, update the builder image

echo "$0: building the image"

cd /tmp/repository/$PATH_TO_DOCKER_FOLDER

OUTPUT_IMAGE=ctumrs/${IMAGE_NAME}:$CUSTOM_TAG

docker buildx create --name container --driver=docker-container --use

STABLE_TAG=""

if [[ "$PPA_VARIANT" == "stable" ]]; then
  STABLE_TAG="--tag ${OUTPUT_IMAGE}_$(date +%y_%m_%d)"
fi

docker buildx build . --file Dockerfile --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg PPA_VARIANT=${PPA_VARIANT} --tag ${OUTPUT_IMAGE} $STABLE_TAG --platform=$PLATFORM --progress plain --push
