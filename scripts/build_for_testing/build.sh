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
DOCKER_IMAGE=$3
ARTIFACTS_FOLDER=$4

[ -z $RUN_LOCALLY ] && RUN_LOCALLY=false

# default for testing

[ -z $LIST ] && LIST=mrs
[ -z $VARIANT ] && VARIANT=testing
[ -z $DOCKER_IMAGE ] && DOCKER_IMAGE=jazzy_builder
[ -z $ARTIFACTS_FOLDER ] && ARTIFACTS_FOLDER=/tmp/artifacts

## | ---------------------- derived args ---------------------- |

# determine our architecture
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

YAML_FILE=$REPO_PATH/${LIST}.yaml

# needed for building open_vins
export ROS_VERSION=2

FULL_COVERAGE_REPOS=$($REPO_PATH/scripts/helpers/parse_yaml.py $YAML_FILE $ARCH)

[ -e /tmp/workspace ] && sudo rm -rf /tmp/workspace
mkdir -p /tmp/workspace/src

echo "$FULL_COVERAGE_REPOS" | while IFS= read -r REPO; do

  cd /tmp/workspace/src

  PACKAGE=$(echo "$REPO" | awk '{print $1}')
  URL=$(echo "$REPO" | awk '{print $2}')
  TEST=$(echo "$REPO" | awk '{print $6}')
  FULL_COVERAGE=$(echo "$REPO" | awk '{print $7}')
  GITMAN=$(echo "$REPO" | awk '{print $8}')

  if [[ "$VARIANT" == "stable" ]]; then
    BRANCH=$(echo "$REPO" | awk '{print $3}')
  elif [[ "$VARIANT" == "testing" ]]; then
    BRANCH=$(echo "$REPO" | awk '{print $4}')
  elif [[ "$VARIANT" == "unstable" ]]; then
    BRANCH=$(echo "$REPO" | awk '{print $5}')
  fi

  if [[ "$TEST" != "True" ]]; then
    continue
  fi

  if [[ "$FULL_COVERAGE" != "True" ]]; then
    continue
  fi

  # if $RUN_LOCALLY && [[ "mrs_uav_managers" != "$PACKAGE" ]]; then
  #   continue
  # fi

  echo "$0: cloning '$URL --depth 1 --branch $BRANCH' into '$PACKAGE'"
  git clone $URL --recurse-submodules --shallow-submodules --depth 1 --branch $BRANCH $PACKAGE

  if [[ "$GITMAN" == "True" ]]; then
    cd $PACKAGE
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

docker buildx use default

echo "$0: loading cached builder docker image"

if ! $RUN_LOCALLY; then

  docker pull ghcr.io/ctu-mrs/buildfarm2:$DOCKER_IMAGE
  docker tag ghcr.io/ctu-mrs/buildfarm2:$DOCKER_IMAGE $DOCKER_IMAGE

fi

echo "$0: image loaded"

mkdir -p /tmp/other_files

cp $MY_PATH/entrypoint.sh /tmp/other_files/entrypoint.sh

## | ---------------------- run the test ---------------------- |

docker run \
  --rm \
  -v /tmp/workspace:/tmp/workspace \
  -v /tmp/other_files:/etc/docker/other_files \
  $DOCKER_IMAGE \
  /bin/bash -c "/etc/docker/other_files/entrypoint.sh"

# tar the workspace

cd /tmp
tar -cvzf workspace.tar.gz workspace
[ ! -e $ARTIFACTS_FOLDER ] && mkdir -p $ARTIFACTS_FOLDER || echo ""
mv workspace.tar.gz $ARTIFACTS_FOLDER/

echo "$0: "
echo "$0: artifacts are:"

ls $ARTIFACTS_FOLDER
