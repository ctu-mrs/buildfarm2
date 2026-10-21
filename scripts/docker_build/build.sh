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
  [ -e repository ] && rm -rf repository || git clone $URL --recurse-submodules --shallow-submodules --depth 1 --branch $BRANCH repository

  if [[ "$GITMAN" == "True" ]]; then
    cd repository
    pipx install gitman==3.5.2 --pip-args regex==2024.9.11
    [[ -e .gitman.yml || -e .gitman.yaml ]] && gitman install
  fi

done < <(echo "$REPOS")

echo "$0: repository cloned to /tmp/repository"

## | ------------------ check if rebuild needed ----------------- |

DOCKER_CHECK_REPO="$IMAGE_NAME"
[ "$IMAGE_NAME" == "bake" ] && DOCKER_CHECK_REPO="$REPOSITORY"

NEED_REBUILD=false

if $RUN_LOCALLY || [[ "${GITHUB_EVENT_NAME:-}" == "workflow_dispatch" ]]; then
  echo "$0: Manual/local build requested, going to build"
  NEED_REBUILD=true
elif [[ "$(date +%u)" == [67] ]]; then
  echo "$0: Weekend run - triggering weekly full rebuild for updated toolchains and base images"
  NEED_REBUILD=true
else
  HUB_TAG_INFO=$(curl -s "https://hub.docker.com/v2/repositories/ctumrs/${DOCKER_CHECK_REPO}/tags/${CUSTOM_TAG}")
  IMAGE_PUSHED_AT=$(echo "$HUB_TAG_INFO" | jq -r .tag_last_pushed 2>/dev/null || true)

  if [ -z "$IMAGE_PUSHED_AT" ] || [ "$IMAGE_PUSHED_AT" = "null" ]; then
    echo "$0: Image ctumrs/${DOCKER_CHECK_REPO}:${CUSTOM_TAG} not found on Docker Hub, going to build"
    NEED_REBUILD=true
  else
    IMAGE_TIME=$(date -d "$IMAGE_PUSHED_AT" +%s 2>/dev/null || echo 0)
    REPO_TIME=$(git -C /tmp/repository log -1 --format=%ct)

    if [ "$REPO_TIME" -gt "$IMAGE_TIME" ]; then
      echo "$0: Repository has newer commit ($REPO_TIME > $IMAGE_TIME), going to build"
      NEED_REBUILD=true
    fi

    AUTH_HEADER=()
    if [ -n "${PUSH_TOKEN:-}" ]; then
      AUTH_HEADER=(-H "Authorization: token $PUSH_TOKEN")
    fi
    PPA_COMMITS=$(curl -s "${AUTH_HEADER[@]}" "https://api.github.com/repos/ctu-mrs/ppa2-${PPA_VARIANT}/commits/debs")
    PPA_DATE=$(echo "$PPA_COMMITS" | jq -r .commit.committer.date 2>/dev/null || true)
    if [ -n "$PPA_DATE" ] && [ "$PPA_DATE" != "null" ]; then
      PPA_TIME=$(date -d "$PPA_DATE" +%s 2>/dev/null || echo 0)
      if [ "$PPA_TIME" -gt "$IMAGE_TIME" ]; then
        echo "$0: PPA ppa2-${PPA_VARIANT} updated ($PPA_TIME > $IMAGE_TIME), going to build"
        NEED_REBUILD=true
      fi
    fi
  fi
fi

if ! $NEED_REBUILD; then
  echo "$0: Image ctumrs/${DOCKER_CHECK_REPO}:${CUSTOM_TAG} is up to date (last pushed at $IMAGE_PUSHED_AT). Skipping."
  exit 0
fi

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

docker buildx create --name container --driver=docker-container --use

if [[ "$IMAGE_NAME" == "bake" ]]; then
  PPA_VARIANT=${PPA_VARIANT} \
  BASE_IMAGE=${BASE_IMAGE} \
  PLATFORM=${PLATFORM} \
  docker buildx bake --progress plain --push
else
  OUTPUT_IMAGE=ctumrs/${IMAGE_NAME}:$CUSTOM_TAG
  STABLE_TAG=""

  if [[ "$PPA_VARIANT" == "stable" ]]; then
    STABLE_TAG="--tag ${OUTPUT_IMAGE}_$(date +%y_%m_%d)"
  fi

  docker buildx build . --file Dockerfile --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg PPA_VARIANT=${PPA_VARIANT} --tag ${OUTPUT_IMAGE} $STABLE_TAG --platform=$PLATFORM --progress plain --push
fi
