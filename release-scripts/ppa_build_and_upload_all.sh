#!/bin/bash
# Build all Ubuntu PPA packages and upload them.
# JLBC, 2013-2020

set -e

# Check:
VERSION_FILE="modules/mola-common/mola-version.cmake"
if [ -f ${VERSION_FILE} ];
then
  # MOLA_VERSION_STR is not set by caller: load it
  MOLA_VERSION_MAJOR=$(grep MOLA_VERSION_NUMBER_MAJOR ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_MINOR=$(grep MOLA_VERSION_NUMBER_MINOR ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_PATCH=$(grep MOLA_VERSION_NUMBER_PATCH ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_STR="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}.${MOLA_VERSION_PATCH}"

	echo "MOLA version: ${MOLA_VERSION_STR}"
else
	echo "ERROR: Run this script from the MOLA root directory."
	exit 1
fi

MOLADIR=`pwd`

# Build PPA packages:
bash release-scripts/prepare_ubuntu_pkgs_for_ppa.sh
cd $HOME/mola_ubuntu
bash $MOLADIR/release-scripts/upload_all_mola_ppa.sh

exit 0
