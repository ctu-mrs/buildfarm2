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

VARIANT_ARG=""
[ -n "$VARIANT" ] && VARIANT_ARG="--variant $VARIANT"

REPOS=$($REPO_PATH/scripts/helpers/parse_yaml_for_docker.py $YAML_FILE $VARIANT_ARG)

echo "$0: ROS package build matrix:" >> /tmp/log.txt 2>&1
echo "$REPOS" >> /tmp/log.txt 2>&1

echo ${REPOS}
