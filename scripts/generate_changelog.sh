#!/usr/bin/env bash

set -x
set -e

# We need a special script since catkin_generate_changelog cannot
# handle git submodules...


SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )


for dir in $SCRIPT_DIR/../modules/*/;
do
GIT_URI=$(cd $dir  && git remote get-url origin)
TMP_DIR=$(mktemp -d)

echo
echo "dir: $(basename $dir) ($GIT_URI) [$TMP_DIR]"
echo "-------------------------------------------------"
(cd $TMP_DIR && git clone $GIT_URI .)
(cd $TMP_DIR && (catkin_generate_changelog || (rm CHANGELOG.rst && catkin_generate_changelog --all)))
gedit $TMP_DIR/CHANGELOG.rst

echo
read -p "CREATE CHANGELOG COMMIT and PUSH [Y/n]? " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
    (cd $TMP_DIR && git commit -am "changelog" && git push)
fi

rm -fr $TMP_DIR || true

done