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
VARIANT=$2
REPOSITORY=$3
PATH_TO_DOCKER_FOLDER=$4

[ -z $RUN_LOCALLY ] && RUN_LOCALLY=false

# default for testing

[ -z $LIST ] && LIST=mrs
[ -z $VARIANT ] && VARIANT=testing
[ -z $REPOSITORY ] && REPOSITORY=mrs_uav_autostart
[ -z $PATH_TO_DOCKER_FOLDER ] && PATH_TO_DOCKER_FOLDER=~/git/realsense/docker

## | ---------------------- derived args ---------------------- |

# determine our architecture
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

ROSDEP_FILE="generated_${LIST}_${ARCH}.yaml"

YAML_FILE=$REPO_PATH/${LIST}.yaml

# needed for building open_vins
export ROS_VERSION=1

REPOS=$($REPO_PATH/scripts/helpers/get_repo_source.py $YAML_FILE $VARIANT $ARCH $REPOSITORY)

# clone and checkout
echo "$REPOS" | while IFS= read -r REPO; do

  cd /tmp

  sudo rm -rf repository

  REPO_NAME=$(echo "$REPO" | awk '{print $1}')
  URL=$(echo "$REPO" | awk '{print $2}')
  BRANCH=$(echo "$REPO" | awk '{print $3}')
  GITMAN=$(echo "$REPO" | awk '{print $4}')

  echo "$0: cloning '$URL --depth 1 --branch $BRANCH' into '$REPO'"
  [ -e repository ] && rm -rf repository || git clone $URL --recurse-submodules --depth 1 --branch $BRANCH repository

  if [[ "$GITMAN" == "True" ]]; then
    cd repository
    pipx install gitman==3.5.2
    [[ -e .gitman.yml || -e .gitman.yaml ]] && gitman install
  fi

done

echo "$0: repository cloned to /tmp/repository"

## --------------------------------------------------------------
## |                        docker build                        |
## --------------------------------------------------------------

$REPO_PATH/ci_scripts/helpers/wait_for_docker.sh

if ! $RUN_LOCALLY; then

  echo "$0: logging in to docker registry"

  echo $PUSH_TOKEN | docker login ghcr.io -u ctumrsbot --password-stdin

fi

## | ---------------------- run the test ---------------------- |

# if there are any artifacts, update the builder image

echo "$0: building the image"

cd $PATH_TO_DOCKER_FOLDER

OUTPUT_IMAGE=ctumrs/${REPO_PATH}:unstable

docker buildx build . --file Dockerfile --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg PPA_VARIANT=${PPA_VARIANT} --tag ${OUTPUT_IMAGE} --progress plain --push
