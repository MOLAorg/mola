#!/bin/bash

MODS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $MODS_DIR

./git-checkout-all-default-branch.sh

