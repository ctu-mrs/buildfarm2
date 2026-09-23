#!/bin/bash

# Source this from a container entrypoint before its first apt-get update.
add_private_ppa() (
  set -e

  [[ -z ${PRIVATE_PPA_TOKEN:-} ]] && return 0

  if [[ ! $PRIVATE_PPA_TOKEN =~ ^([^:[:space:]]+):([^@[:space:]]+)@([A-Za-z0-9.-]+):([0-9]+)$ ]]; then
    echo 'Invalid PRIVATE_PPA_TOKEN format (expected user:password@domain:port)' >&2
    return 1
  fi

  ppa_user=${BASH_REMATCH[1]}
  ppa_password=${BASH_REMATCH[2]}
  ppa_domain=${BASH_REMATCH[3]}
  ppa_port=${BASH_REMATCH[4]}
  ppa_host="${ppa_domain}:${ppa_port}"

  install -d -m 755 /etc/apt/keyrings /etc/apt/auth.conf.d /etc/apt/sources.list.d
  umask 077
  ppa_netrc=$(mktemp)
  ppa_key=$(mktemp)
  trap 'rm -f "$ppa_netrc" "$ppa_key"' EXIT
  printf 'machine %s login %s password %s\n' "$ppa_domain" "$ppa_user" "$ppa_password" > "$ppa_netrc"

  curl --fail --silent --show-error --netrc-file "$ppa_netrc" \
    "https://${ppa_host}/public.key" -o "$ppa_key"
  gpg --batch --yes --dearmor -o /etc/apt/keyrings/mrs-ppa-private.gpg "$ppa_key"
  chmod 644 /etc/apt/keyrings/mrs-ppa-private.gpg

  printf 'machine %s login %s password %s\n' \
    "$ppa_host" "$ppa_user" "$ppa_password" > /etc/apt/auth.conf.d/mrs-ppa-private.conf
  printf 'deb [signed-by=/etc/apt/keyrings/mrs-ppa-private.gpg] https://%s/ %s main\n' \
    "$ppa_host" "$(lsb_release -cs)" > /etc/apt/sources.list.d/ctu-mrs-private.list
  chmod 644 /etc/apt/sources.list.d/ctu-mrs-private.list
)

add_private_ppa
