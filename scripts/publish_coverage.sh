#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

REPO_PATH=$MY_PATH/..

## | -------------------------- args -------------------------- |

# INPUTS
LIST=$1
VARIANT=$2
ARTIFACT_FOLDER=$3

[ -z $RUN_LOCALLY ] && RUN_LOCALLY=false

# default for testing

[ -z $LIST ] && LIST=mrs
[ -z $VARIANT ] && VARIANT=unstable
[ -z $ARTIFACT_FOLDER ] && ARTIFACT_FOLDER=/tmp/artifacts

# determine our architecture
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

## | ---------------------- derived args ---------------------- |

echo "$0: creating workspace"

WORKSPACE=/tmp/workspace

mkdir -p $WORKSPACE/src

YAML_FILE=$REPO_PATH/${LIST}.yaml

FULL_COVERAGE_REPOS=$($REPO_PATH/scripts/helpers/parse_yaml.py $YAML_FILE $ARCH)

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

echo "$0: repositories cloned to /tmp/repository"

echo "$0: workspace extracted"

ls -la $WORKSPACE/src

# are there any coverage files?

$REPO_PATH/ci_scripts/helpers/wait_for_docker.sh

docker pull klaxalk/lcov

echo "$0: combining coverage report files"

ARGS=""

for file in `ls $ARTIFACT_FOLDER | grep ".json"`; do

  if [ -s ${ARTIFACT_FOLDER}/${file} ]; then
    ARGS="${ARGS} -a ${ARTIFACT_FOLDER}/${file}"
  fi

done

docker run \
  --rm \
  -v $WORKSPACE:/tmp/workspace \
  -v $ARTIFACT_FOLDER:/tmp/artifacts \
  klaxalk/lcov \
  /bin/bash -c "cd /tmp/workspace && gcovr $ARGS --filter='^.*\/(src|include|tests?)\/.*$' --exclude='^.*\/eth_trajectory_generation\/.*$' --lcov /tmp/workspace/coverage.info"

echo "$0: filtering coverage report"

docker run \
  --rm \
  -v $WORKSPACE:/tmp/workspace \
  -v $ARTIFACT_FOLDER:/tmp/artifacts \
  klaxalk/lcov \
  /bin/bash -c "lcov -a /tmp/workspace/coverage.info --ignore-errors unused,unused,inconsistent,inconsistent,corrupt,corrupt,unsupported,unsupported,format,format --demangle-cpp --erase-functions '.*(\{lambda\(|anonymous\snamespace).*' --output-file /tmp/workspace/coverage2.info || echo 'coverage tracefile is empty'"

echo "$0: generating coverage html report"

docker run \
  --rm \
  -v $WORKSPACE:/tmp/workspace \
  -v $ARTIFACT_FOLDER:/tmp/artifacts \
  klaxalk/lcov \
  /bin/bash -c "genhtml -c /color_fix.css --title 'MRS UAV System - Test coverage report' --prefix '/tmp/workspace/src' --ignore-errors unused,unused,inconsistent,inconsistent,corrupt,corrupt,unsupported,unsupported,format,format --demangle-cpp --legend --frames --show-details -o /tmp/artifacts/coverage_html /tmp/workspace/coverage2.info | tee /tmp/workspace/coverage.log"

echo "$0: generating coverage badge"

COVERAGE_PCT=`cat $WORKSPACE/coverage.log | grep -E "lines\.\.\." | awk '{print $2}'`

echo "Coverage: $COVERAGE_PCT"

docker pull klaxalk/pybadges

docker run \
  --rm \
  -v $WORKSPACE:/tmp/workspace \
  -v $ARTIFACT_FOLDER:/tmp/artifacts \
  klaxalk/pybadges \
  /bin/bash -c "/bin/python3 -m pybadges --left-text='test coverage' --right-text='${COVERAGE_PCT}' --right-color='#0c0' > /tmp/artifacts/coverage_html/badge.svg"

echo "$0: coverage generation finished"
