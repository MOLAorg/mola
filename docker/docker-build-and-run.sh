#!/bin/bash

#export UID=$(id -u)
export GID=$(id -g)
xhost +local:docker

export MOLA_DATASETS_HOST=${MOLA_DATASETS:-./my_datasets}
export MOLA_DATASETS_CONTAINER=${MOLA_DATASETS:-/home/rosuser/data}

export BUILDKIT_PROGRESS=plain

# use this to force re-download the latest version of all git repositories:
# docker compose build --no-cache

docker compose up --build -d $1

