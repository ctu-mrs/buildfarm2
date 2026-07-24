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
BASE_IMAGE=$4
DOCKER_IMAGE=$5
ARTIFACTS_FOLDER=$6

[ -z $RUN_LOCALLY ] && RUN_LOCALLY=false

# when building in parallel groups, the builder image is updated once per group
# instead, so that the jobs within a group don't race pushing the same tag
[ -z $SKIP_BUILDER_UPDATE ] && SKIP_BUILDER_UPDATE=false

# default for testing

[ -z $LIST ] && LIST=mrs
[ -z $VARIANT ] && VARIANT=testing
[ -z $REPOSITORY ] && REPOSITORY=mrs_uav_autostart
[ -z $BASE_IMAGE ] && BASE_IMAGE=ctumrs/ros_jazzy:latest
[ -z $DOCKER_IMAGE ] && DOCKER_IMAGE=jazzy_builder
[ -z $ARTIFACTS_FOLDER ] && ARTIFACTS_FOLDER=/tmp/artifacts

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
    pipx install gitman==3.5.2 --pip-args regex==2024.9.11
    [[ -e .gitman.yml || -e .gitman.yaml ]] && gitman install
  fi

done

echo "$0: repository cloned to /tmp/repository"

## | -------------------- fast skip logic --------------------- |

if [ ! -e $ARTIFACTS_FOLDER/compiled.txt ]; then
  touch $ARTIFACTS_FOLDER/compiled.txt
fi

echo "$0: evaluating fast skip logic"

# Add the PPA to the runner to use apt-cache policy natively
curl -sL https://ctu-mrs.github.io/ppa2-${VARIANT}/add_sources_ppa.sh | bash

BUILD_ORDER=$($REPO_PATH/ci_scripts/helpers/get_package_build_order.py /tmp/repository)

OLDIFS=$IFS; IFS=$'\n'; for LINE in $BUILD_ORDER; do

  PACKAGE=$(echo $LINE | awk '{print $1}')
  PKG_PATH_SUB=$(echo $LINE | awk '{print $2}')

  echo "$0: cding to '/tmp/repository/$PKG_PATH_SUB'"
  cd /tmp/repository/$PKG_PATH_SUB

  FUTURE_DEB_NAME=$(echo "ros-jazzy-$PACKAGE" | sed 's/_/-/g')

  echo "$0: future deb name: $FUTURE_DEB_NAME"

  # even a package that we end up skipping has to stay resolvable by rosdep
  ROSDEP_ENTRIES="$ROSDEP_ENTRIES$PACKAGE:
    ubuntu: [$FUTURE_DEB_NAME]
"

  SHA=$(git rev-parse --short HEAD)
  DOCKER_SHA=$(cat $ARTIFACTS_FOLDER/base_sha.txt)

  echo "$0: SHA=$SHA"

  GIT_SHA_MATCHES=$(apt-cache policy $FUTURE_DEB_NAME | grep "Candidate" | grep "git.${SHA}" | wc -l)
  ON_PUSH_BUILD=$(apt-cache policy $FUTURE_DEB_NAME | grep "Candidate" | grep "on.push.build" | wc -l)
  DOCKER_SHA_MATCHES=$(apt-cache policy $FUTURE_DEB_NAME | grep "Candidate" | grep "base.${DOCKER_SHA}" | wc -l)

  NEW_COMMIT=false
  if [[ "$GIT_SHA_MATCHES" == "0" ]] || [ "$ON_PUSH_BUILD" -ge "1" ]; then
    echo "$0: new commit detected, going to compile"
    NEW_COMMIT=true
  fi

  MY_DEPENDENCIES=$($REPO_PATH/ci_scripts/helpers/get_package_dependencies.py /tmp/repository/$PKG_PATH_SUB)

  echo ""
  echo "$0: MY_DEPENDENCIES: '$MY_DEPENDENCIES'"
  echo ""

  DEPENDENCIES_CHANGED=false

  # this \|/ has to iterate over the dependencies in bash
  readarray -t MY_DEPENDENCIES_ITEMIZED <<< "$MY_DEPENDENCIES"
  for dep in "${MY_DEPENDENCIES_ITEMIZED[@]}"; do

    # skip empty strings (readarray produces one empty element when input is empty)
    if [[ -z "$dep" ]]; then
      continue
    fi

    FOUND=$(grep -x "$dep" "$ARTIFACTS_FOLDER/compiled.txt" | wc -l)

    echo "$0: checking if '$dep' is within MY_DEPENDENCIES, FOUND='$FOUND'"

    if [ $FOUND -ge 1 ]; then
      DEPENDENCIES_CHANGED=true
      echo "$0: The dependency $dep has been updated, going to compile"
    else
      echo "$0: ... nope"
    fi

  done

  if [[ "$DOCKER_SHA_MATCHES" == "0" && "$(date +%u)" == [67] ]]; then
    echo "$0: weekly rebuild with new base image, going to compile"
    DEPENDENCIES_CHANGED=true
  fi

done; IFS=$OLDIFS

if ! $DEPENDENCIES_CHANGED && ! $NEW_COMMIT; then
  echo "$0: Skipping"

  # nothing is compiled here, so the entries are carried over for the deploy
  echo "$ROSDEP_ENTRIES" >> $ARTIFACTS_FOLDER/$ROSDEP_FILE

  exit 0
fi

cd /tmp

## | ---------------------- docker build ---------------------- |

$REPO_PATH/ci_scripts/helpers/wait_for_docker.sh

if ! $RUN_LOCALLY; then

  echo "$0: logging in to docker registry"

  echo $PUSH_TOKEN | docker login ghcr.io -u ctumrsbot --password-stdin

fi

TRANSPORT_IMAGE=alpine:latest

docker buildx use default

echo "$0: loading cached builder docker image"

if ! $RUN_LOCALLY; then

  docker pull ghcr.io/ctu-mrs/buildfarm2:$DOCKER_IMAGE
  docker tag ghcr.io/ctu-mrs/buildfarm2:$DOCKER_IMAGE $DOCKER_IMAGE

fi

echo "$0: image loaded"

if [ ! -e $ARTIFACTS_FOLDER/$ROSDEP_FILE ]; then
  touch $ARTIFACTS_FOLDER/$ROSDEP_FILE
fi

mkdir -p /tmp/debs
mkdir -p /tmp/other_files

cp $ARTIFACTS_FOLDER/base_sha.txt /tmp/other_files/base_sha.txt
cp $MY_PATH/entrypoint.sh /tmp/other_files/entrypoint.sh

mv $ARTIFACTS_FOLDER/compiled.txt /tmp/other_files/compiled.txt
mv $ARTIFACTS_FOLDER/$ROSDEP_FILE /tmp/other_files/rosdep.yaml

echo "$BUILD_ORDER" > /tmp/other_files/build_order.txt
cp $REPO_PATH/ci_scripts/helpers/get_package_dependencies.py /tmp/other_files/get_package_dependencies.py

echo "$0:"
echo "$0: builder order:"
cat /tmp/other_files/build_order.txt
echo "$0: "

## | ---------------------- run the test ---------------------- |

docker run \
  --rm \
  -v /tmp/repository:/etc/docker/repository \
  -v /tmp/debs:/etc/docker/debs \
  -v /tmp/other_files:/etc/docker/other_files \
  $DOCKER_IMAGE \
  /bin/bash -c "/etc/docker/other_files/entrypoint.sh"

# if there are any artifacts, update the builder image

DEBS_EXIST=$(ls /tmp/debs | grep ".deb" | wc -l)

if [ $DEBS_EXIST -gt 0 ]; then

  if ! $SKIP_BUILDER_UPDATE; then
    $MY_PATH/update_builder.sh /tmp/debs $BASE_IMAGE $DOCKER_IMAGE
  fi

  echo "$0: copying artifacts"

  mv /tmp/debs/* $ARTIFACTS_FOLDER/

fi

# copy the artifacts for the next build job
mv /tmp/other_files/rosdep.yaml $ARTIFACTS_FOLDER/$ROSDEP_FILE
mv /tmp/other_files/compiled.txt $ARTIFACTS_FOLDER/compiled.txt

echo "$0: "
echo "$0: artifacts are:"

ls $ARTIFACTS_FOLDER
