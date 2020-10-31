#!/bin/bash

# See https://reproducible-builds.org/specs/source-date-epoch/
# get SOURCE_DATE_EPOCH with UNIX time_t
if [ -d ".git" ];
then
  SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)
else
  echo "Error: intended for use from within a git repository"
  exit 1
fi
MOLA_SNAPSHOT_VERSION=$(date -d @$SOURCE_DATE_EPOCH +%Y%m%d-%H%M)

MOLA_SNAPSHOT_VERSION+="-git-"
MOLA_SNAPSHOT_VERSION+=`git rev-parse --short=8 HEAD`
MOLA_SNAPSHOT_VERSION+="-"
