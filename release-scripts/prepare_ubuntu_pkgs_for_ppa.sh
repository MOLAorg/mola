#!/bin/bash
# Creates a set of packages for each different Ubuntu distribution, with the
# intention of uploading them to:
#   https://launchpad.net/~joseluisblancoc/+archive/mola
#
# JLBC, 2010-2020

set -e

# List of distributions to create PPA packages for:
LST_DISTROS=(bionic focal groovy)

# Checks
# --------------------------------
VERSION_FILE="modules/mola-common/mola-version.cmake"
if [ -f ${VERSION_FILE} ];
then
  MOLA_VERSION_MAJOR=$(grep MOLA_VERSION_NUMBER_MAJOR ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_MINOR=$(grep MOLA_VERSION_NUMBER_MINOR ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_PATCH=$(grep MOLA_VERSION_NUMBER_PATCH ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_STR="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}.${MOLA_VERSION_PATCH}"

  MOLA_VER_MM="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}"
  MOLA_VER_MMP="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}.${MOLA_VERSION_PATCH}"
  echo "MOLA version: ${MOLA_VER_MMP}"
else
  echo "ERROR: Run this script from the MOLA root directory."
  exit 1
fi

if [ -z "${MOLA_UBUNTU_OUT_DIR}" ]; then
  export MOLA_UBUNTU_OUT_DIR="$HOME/mola_ubuntu"
fi
MOLASRC=`pwd`
if [ -z "${MOLA_DEB_DIR}" ]; then
  export MOLA_DEB_DIR="$HOME/mola_debian"
fi
MOLA_EXTERN_DEBIAN_DIR="$MOLASRC/release-scripts/debian/"
EMAIL4DEB="Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>"

# Clean out dirs:
rm -fr $MOLA_UBUNTU_OUT_DIR/

# -------------------------------------------------------------------
# And now create the custom packages for each Ubuntu distribution:
# -------------------------------------------------------------------
count=${#LST_DISTROS[@]}
IDXS=$(seq 0 $(expr $count - 1))

cp ${MOLA_EXTERN_DEBIAN_DIR}/changelog /tmp/my_changelog

for IDX in ${IDXS};
do
  DEBIAN_DIST=${LST_DISTROS[$IDX]}

  # -------------------------------------------------------------------
  # Call the standard "prepare_debian.sh" script:
  # -------------------------------------------------------------------
  cd ${MOLASRC}
  bash release-scripts/prepare_debian.sh -s -d ${DEBIAN_DIST} ${EMBED_EIGEN_FLAG}

  CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  source $CUR_SCRIPT_DIR/prepare_debian_gen_snapshot_version.sh # populate MOLA_SNAPSHOT_VERSION

  echo "===== Distribution: ${DEBIAN_DIST}  ========="
  cd ${MOLA_DEB_DIR}/mola-${MOLA_VER_MMP}~snapshot${MOLA_SNAPSHOT_VERSION}${DEBIAN_DIST}/debian
  #cp ${MOLA_EXTERN_DEBIAN_DIR}/changelog changelog
  cp /tmp/my_changelog changelog
  DEBCHANGE_CMD="--newversion ${MOLA_VERSION_STR}~snapshot${MOLA_SNAPSHOT_VERSION}${DEBIAN_DIST}-1"
  echo "Changing to a new Debian version: ${DEBCHANGE_CMD}"
  echo "Adding a new entry to debian/changelog for distribution ${DEBIAN_DIST}"
  DEBEMAIL=${EMAIL4DEB} debchange $DEBCHANGE_CMD -b --distribution ${DEBIAN_DIST} --force-distribution New version of upstream sources.

  cp changelog /tmp/my_changelog

  echo "Now, let's build the source Deb package with 'debuild -S -sa':"
  cd ..
  # -S: source package
  # -sa: force inclusion of sources
  # -d: don't check dependencies in this system
  debuild -S -sa -d

  # Make a copy of all these packages:
  cd ..
  mkdir -p $MOLA_UBUNTU_OUT_DIR/$DEBIAN_DIST
  cp mola_* $MOLA_UBUNTU_OUT_DIR/$DEBIAN_DIST/
  echo ">>>>>> Saving packages to: $MOLA_UBUNTU_OUT_DIR/$DEBIAN_DIST/"
done

exit 0
