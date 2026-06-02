#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

ARTIFACTS_FOLDER=$1
BASE_IMAGE=$2
REPO_FOLDER=/etc/docker/repository

cd "$REPO_FOLDER"

git config --global --add safe.directory "$REPO_FOLDER"

# call the build script within the clone repository
./.ci/build_package.sh ${ARTIFACTS_FOLDER} ${BASE_IMAGE}

# add the repo url and commit id to the debian control file of each generated deb
REPO_URL=$(git -C "$REPO_FOLDER" config --get remote.origin.url | sed -E 's#^ssh://git@([^/:]+)(:[0-9]+)?/#https://\1/#; s#^git@([^:]+):#https://\1/#; s#\.git$##')
HOMEPAGE_URL="$REPO_URL/tree/$(git -C "$REPO_FOLDER" rev-parse HEAD)"

for DEB in "${ARTIFACTS_FOLDER}"/*.deb; do
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