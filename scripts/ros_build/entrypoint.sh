#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

VARIANT=$1

# determine our architecture
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

DEBS_FOLDER=/etc/docker/debs
REPO_FOLDER=/etc/docker/repository
OTHER_FILES_FOLDER=/etc/docker/other_files

chmod +x $OTHER_FILES_FOLDER/get_package_dependencies.py

git config --global --add safe.directory $REPO_FOLDER

BUILD_ORDER=$(cat /etc/docker/other_files/build_order.txt)

echo ""
echo "$0: topological build order:"
echo "$BUILD_ORDER"
echo ""

ROSDEP_FILE=$OTHER_FILES_FOLDER/rosdep.yaml

cat $ROSDEP_FILE

if [ -s $ROSDEP_FILE ]; then

  echo "$0: adding $ROSDEP_FILE to rosdep"
  echo "$0: contents:"

  echo "yaml file://$ROSDEP_FILE" | tee /etc/ros/rosdep/sources.list.d/temp.list

  rosdep update

fi

OLDIFS=$IFS; IFS=$'\n'; for LINE in $BUILD_ORDER; do

  PACKAGE=$(echo $LINE | awk '{print $1}')
  PKG_PATH=$(echo $LINE | awk '{print $2}')

  echo "$0: cding to '$REPO_FOLDER/$PKG_PATH'"
  cd $REPO_FOLDER/$PKG_PATH

  FUTURE_DEB_NAME=$(echo "ros-jazzy-$PACKAGE" | sed 's/_/-/g')

  echo "$0: future deb name: $FUTURE_DEB_NAME"

  git config --global --add safe.directory $REPO_FOLDER/$PKG_PATH

  SHA=$(git rev-parse --short HEAD)
  DOCKER_SHA=$(cat $OTHER_FILES_FOLDER/base_sha.txt)

  echo "$0: SHA=$SHA"

  GIT_SHA_MATCHES=$(apt-cache policy $FUTURE_DEB_NAME | grep "Candidate" | grep "git.${SHA}" | wc -l)
  ON_PUSH_BUILD=$(apt-cache policy $FUTURE_DEB_NAME | grep "Candidate" | grep "on.push.build" | wc -l)
  DOCKER_SHA_MATCHES=$(apt-cache policy $FUTURE_DEB_NAME | grep "Candidate" | grep "base.${DOCKER_SHA}" | wc -l)

  NEW_COMMIT=false
  if [[ "$GIT_SHA_MATCHES" == "0" ]] || [ "$ON_PUSH_BUILD" -ge "1" ]; then
    echo "$0: new commit detected, going to compile"
    NEW_COMMIT=true
  fi

  MY_DEPENDENCIES=$($OTHER_FILES_FOLDER/get_package_dependencies.py $REPO_FOLDER/$PKG_PATH)

  echo ""
  echo "$0: MY_DEPENDENCIES: '$MY_DEPENDENCIES'"
  echo ""

  DEPENDENCIES_CHANGED=false

  # this \|/ has to iterate over the dependencies in bash
  readarray -t MY_DEPENDENCIES_ITEMIZED <<< "$MY_DEPENDENCIES"
  for dep in "${MY_DEPENDENCIES_ITEMIZED[@]}"; do

    FOUND=$(cat $OTHER_FILES_FOLDER/compiled.txt | grep $dep | wc -l)

    echo "$0: checking if '$dep' is within MY_DEPENDENCIES, FOUND='$FOUND'"

    if [ $FOUND -ge 1 ]; then
      DEPENDENCIES_CHANGED=true
      echo "$0: The dependency $dep has been updated, going to compile"
    fi

    echo "$0: ... nope"

  done

  if [[ "$DOCKER_SHA_MATCHES" == "0" ]]; then
    echo "$0: base image changed, going to compile"
    DEPENDENCIES_CHANGED=true
  fi

  if $DEPENDENCIES_CHANGED || $NEW_COMMIT; then

    ## don't run if COLCON_IGNORE is present

    [ -e $PKG_PATH/COLCON_IGNORE ] && continue

    apt-get -y update

    rosdep install -y -v --rosdistro=jazzy --dependency-types=build --dependency-types=buildtool --from-paths ./

    source /opt/ros/jazzy/setup.bash

    echo "$0: Running bloom on a package in '$PKG_PATH'"

    if [[ "$ARCH" != "arm64" ]]; then
      export DEB_BUILD_OPTIONS="parallel=`nproc`"
    fi

    bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro jazzy

    epoch=2
    build_flag="$(date +%Y%m%d.%H%M%S)~git.$SHA.base.$DOCKER_SHA"

    sed -i "s/(/($epoch:/" ./debian/changelog
    sed -i "s/)/.${build_flag})/" ./debian/changelog

    if [[ "$ARCH" != "arm64" ]]; then
      fakeroot debian/rules "binary --parallel"
    else
      fakeroot debian/rules "binary"
    fi

    DEB_NAME=$(dpkg --field ../*.deb | grep "Package:" | head -n 1 | awk '{print $2}')

    DEBS=(../*.deb)

    echo "$0: installing newly compiled deb file"
    [ -e "${DEBS[0]}" ] && apt-get -y install --allow-downgrades ../*.deb || echo "$0: no artifacts to be installed"

    echo "$0: moving the artifact to $DEBS_FOLDER"
    [ -e "${DEBS[0]}" ] && mv ../*.deb $DEBS_FOLDER || echo "$0: no artifacts to be moved"

    echo "$PACKAGE:
    ubuntu: [$DEB_NAME]
  " >> $ROSDEP_FILE

    rosdep update

    source /opt/ros/jazzy/setup.bash

    echo "$PACKAGE" >> $OTHER_FILES_FOLDER/compiled.txt

  else

    echo "$0: not building this package, the newest version is already in the PPA"

    echo "$PACKAGE:
    ubuntu: [$FUTURE_DEB_NAME]
  " >> $ROSDEP_FILE

  fi

done

echo ""
echo "$0: the generated rosdep contains:"
echo ""
cat $ROSDEP_FILE
echo ""
