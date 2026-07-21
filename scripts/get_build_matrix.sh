#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?, log:" && cat /tmp/log.txt' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

REPO_PATH=${MY_PATH}/..

## | ------------------------ arguments ----------------------- |

LIST=$1
VARIANT=$2

## | ----------------------------  ---------------------------- |

YAML_FILE=$REPO_PATH/$LIST.yaml

ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

REPOS=$($REPO_PATH/scripts/helpers/parse_yaml.py $YAML_FILE $ARCH)

WORKSPACE=/tmp/workspace

if [ -e $WORKSPACE ]; then
  rm -rf $WORKSPACE
fi

mkdir -p $WORKSPACE >> /tmp/log.txt 2>&1

cd $WORKSPACE >> /tmp/log.txt 2>&1

# clone and checkout
echo "$REPOS" | while IFS= read -r REPO; do

  cd $WORKSPACE >> /tmp/log.txt 2>&1

  PACKAGE=$(echo "$REPO" | awk '{print $1}')
  URL=$(echo "$REPO" | awk '{print $2}')

  if [[ "$VARIANT" == "stable" ]]; then
    BRANCH=$(echo "$REPO" | awk '{print $3}')
  elif [[ "$VARIANT" == "testing" ]]; then
    BRANCH=$(echo "$REPO" | awk '{print $4}')
  else
    BRANCH=$(echo "$REPO" | awk '{print $5}')
  fi

  if [[ "$BRANCH" == "none" ]]; then
    continue
  fi

  echo "$0: Cloning '$REPO' from '$URL --branch $BRANCH' into '$PACKAGE'" >> /tmp/log.txt 2>&1

  git clone $URL --recurse-submodules --shallow-submodules --depth 1 --branch $BRANCH $PACKAGE >> /tmp/log.txt 2>&1

done

echo "$0: Done cloning" >> /tmp/log.txt 2>&1
echo "" >> /tmp/log.txt 2>&1

BUILD_ORDER=$($REPO_PATH/scripts/helpers/get_repository_build_order.py $WORKSPACE $YAML_FILE)

echo "$0: ROS package build order:" >> /tmp/log.txt 2>&1
echo "$BUILD_ORDER" >> /tmp/log.txt 2>&1
echo "" >> /tmp/log.txt 2>&1

echo "${BUILD_ORDER}"
