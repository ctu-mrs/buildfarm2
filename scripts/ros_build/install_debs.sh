#!/bin/bash

set -e

cleanup_private_ppa() {
  rm -f /etc/apt/keyrings/mrs-ppa-private.gpg \
    /etc/apt/auth.conf.d/mrs-ppa-private.conf \
    /etc/apt/sources.list.d/ctu-mrs-private.list
}

if [[ -e /run/secrets/PRIVATE_PPA_TOKEN ]]; then
  if [[ ! -s /run/secrets/PRIVATE_PPA_TOKEN ]]; then
    echo 'Private PPA BuildKit secret is empty' >&2
    exit 1
  fi
  trap cleanup_private_ppa EXIT
  PRIVATE_PPA_TOKEN=$(< /run/secrets/PRIVATE_PPA_TOKEN)
  . /tmp/add_private_ppa.sh
  unset PRIVATE_PPA_TOKEN
fi

ls /tmp/tmp_debs
apt-get update
apt-get -y install --no-install-recommends /tmp/tmp_debs/*.deb
rm -rf /tmp/tmp_debs /var/lib/apt/lists/*
