#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# determine our architecture
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

DEBS_FOLDER=/etc/docker/debs
REPO_FOLDER=/etc/docker/repository
OTHER_FILES_FOLDER=/etc/docker/other_files

chmod +x $OTHER_FILES_FOLDER/get_package_dependencies.py

git config --global --add safe.directory $REPO_FOLDER

REPO_URL=$(git -C "$REPO_FOLDER" config --get remote.origin.url | sed -E 's#^ssh://git@([^/:]+)(:[0-9]+)?/#https://\1/#; s#^git@([^:]+):#https://\1/#; s#\.git$##')
HOMEPAGE_URL="$REPO_URL/tree/$(git -C "$REPO_FOLDER" rev-parse HEAD)"

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

  git config --global --add safe.directory $REPO_FOLDER/$PKG_PATH

  SHA=$(git rev-parse --short HEAD)
  DOCKER_SHA=$(cat $OTHER_FILES_FOLDER/base_sha.txt)

  ## don't run if COLCON_IGNORE is present

  if [ -e ./COLCON_IGNORE ]; then

    echo "$0: COLCON_IGNORE present, skipping $PACKAGE"
    continue
  fi

  apt-get -y update

  rosdep install -y -v --rosdistro=jazzy --dependency-types=build --dependency-types=build_export --dependency-types=buildtool --from-paths ./

  source /opt/ros/jazzy/setup.bash

  echo "$0: Running bloom on a package in '$PKG_PATH'"

  export DEB_BUILD_OPTIONS="nocheck"

  if [[ "$ARCH" != "arm64" ]]; then
    export DEB_BUILD_OPTIONS="$DEB_BUILD_OPTIONS parallel=`nproc`"
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

  for DEB in "${DEBS[@]}"; do
    [ -e "$DEB" ] || continue
    TMPDIR=$(mktemp -d)
    echo "$0: modifying homepage in $DEB ($TMPDIR) to $HOMEPAGE_URL"
    dpkg-deb -R "$DEB" "$TMPDIR"
    rm -f $DEB
    if grep -q '^Homepage:' "$TMPDIR/DEBIAN/control"; then
      sed -i "s#^Homepage:.*#Homepage: ${HOMEPAGE_URL}#" "$TMPDIR/DEBIAN/control"
    else
      printf 'Homepage: %s\n' "${HOMEPAGE_URL}" >> "$TMPDIR/DEBIAN/control"
    fi
    printf 'XB-Build-Date: %s\n' "$(date -R)" >> "$TMPDIR/DEBIAN/control"
    dpkg-deb -b "$TMPDIR" "$DEB"
    rm -rf "$TMPDIR"
  done

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

done; IFS=$OLDIFS

echo ""
echo "$0: the generated rosdep contains:"
echo ""
cat $ROSDEP_FILE
echo ""
