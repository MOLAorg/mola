#!/bin/bash

MODS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $MODS_DIR

# Ensure we are not in a detached commit (default after submodules clone):
./git-all checkout master

# Pull:
./git-all pull

