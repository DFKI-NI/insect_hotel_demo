#!/usr/bin/env bash
set -e

FILEPATH=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
cd "$FILEPATH"/.. || exit


sudo apt-get update -qq
sudo apt-get install -qq -y python3-vcstool python3-pip git

# Install demo dependencies.
vcs import --recursive --skip-existing --workers 1 < "$FILEPATH"/my.repos
vcs pull

# Install mobipick labs dependencies.
mobipick_labs/install-deps.sh
