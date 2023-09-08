#!/usr/bin/env bash

set -x
set -e

# See: README.md

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

for dir in $SCRIPT_DIR/../modules/*/;
do
echo
echo "dir: $(basename $dir)"
echo "-------------------------------------------------"
cd $dir
git diff
echo
NEW_VERSION=$(grep "<version>" package.xml | sed -n 's:.*<version>\(.*\)</version>.*:\1:p')

read -p "CREATE NEW VERSION COMMIT for VERSION=$NEW_VERSION [Y/n]? " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
    git commit -am "$NEW_VERSION"
    git push
    git tag $NEW_VERSION -m "$NEW_VERSION"
    git push --tags
fi

done