#!/bin/bash
# Prepare to build a Debian package.
# JLBC, 2008-2020

set -e   # end on error

APPEND_SNAPSHOT_NUM=0
APPEND_LINUX_DISTRO=""
VALUE_EXTRA_CMAKE_PARAMS=""
while getopts "sd:c:" OPTION
do
     case $OPTION in
         s)
             APPEND_SNAPSHOT_NUM=1
             ;;
         d)
             APPEND_LINUX_DISTRO=$OPTARG
             ;;
         c)
             VALUE_EXTRA_CMAKE_PARAMS=$OPTARG
             ;;
         ?)
             echo "Unknown command line argument!"
             exit 1
             ;;
     esac
done

VERSION_FILE="modules/mola-common/mola-version.cmake"
if [ -f ${VERSION_FILE} ];
then
  MOLA_VERSION_MAJOR=$(grep MOLA_VERSION_NUMBER_MAJOR ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_MINOR=$(grep MOLA_VERSION_NUMBER_MINOR ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_PATCH=$(grep MOLA_VERSION_NUMBER_PATCH ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
  MOLA_VERSION_STR="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}.${MOLA_VERSION_PATCH}"

  MOLA_VER_MM="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}"
  MOLA_VER_MMP="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}.${MOLA_VERSION_PATCH}"
else
  echo "ERROR: Run this script from the MOLA root directory."
  exit 1
fi

# Append snapshot?
if [ $APPEND_SNAPSHOT_NUM == "1" ];
then
  CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  source $CUR_SCRIPT_DIR/prepare_debian_gen_snapshot_version.sh  # populate MOLA_SNAPSHOT_VERSION

  MOLA_VERSION_STR="${MOLA_VERSION_STR}~snapshot${MOLA_SNAPSHOT_VERSION}${APPEND_LINUX_DISTRO}"
else
  MOLA_VERSION_STR="${MOLA_VERSION_STR}${APPEND_LINUX_DISTRO}"
fi

# Call prepare_release, which also detects MOLA version and exports it
# in MOLA_VERSION_STR, etc.
VERSION_FILE="modules/mola-common/mola-version.cmake"
if [ -f ${VERSION_FILE} ];
then
  MOLASRC=`pwd`
  source release-scripts/prepare_release.sh
  echo
  echo "## Done prepare_release.sh"
else
  echo "ERROR: Run this script from the MOLA root directory."
  exit 1
fi

echo "=========== Generating MOLA ${MOLA_VER_MMP} Debian package =============="
cd $MOLASRC

set -x
if [ -z "$MOLA_DEB_DIR" ]; then
  MOLA_DEB_DIR="$HOME/mola_debian"
fi
MOLA_EXTERN_DEBIAN_DIR="$MOLASRC/release-scripts/debian/"

if [ -f ${MOLA_EXTERN_DEBIAN_DIR}/control.in ];
then
	echo "Using debian dir: ${MOLA_EXTERN_DEBIAN_DIR}"
else
	echo "ERROR: Cannot find ${MOLA_EXTERN_DEBIAN_DIR}"
	exit 1
fi

MOLA_DEBSRC_DIR=$MOLA_DEB_DIR/mola-${MOLA_VERSION_STR}

echo "MOLA_VERSION_STR: ${MOLA_VERSION_STR}"
echo "MOLA_DEBSRC_DIR: ${MOLA_DEBSRC_DIR}"

# Prepare a directory for building the debian package:
#
rm -fR $MOLA_DEB_DIR || true
mkdir -p $MOLA_DEB_DIR

# Orig tarball:
echo "Copying orig tarball: mola_${MOLA_VERSION_STR}.orig.tar.gz"
cp $HOME/mola_release/mola*.tar.gz $MOLA_DEB_DIR/mola_${MOLA_VERSION_STR}.orig.tar.gz
cp $HOME/mola_release/mola*.tar.gz.asc $MOLA_DEB_DIR/mola_${MOLA_VERSION_STR}.orig.tar.gz.asc
cd ${MOLA_DEB_DIR}
tar -xf mola_${MOLA_VERSION_STR}.orig.tar.gz

if [ ! -d "${MOLA_DEBSRC_DIR}" ];
then
  mv mola-* ${MOLA_DEBSRC_DIR}  # fix different dir names for Ubuntu PPA packages
fi

if [ ! -f "${MOLA_DEBSRC_DIR}/CMakeLists.txt" ];
then
	echo "*ERROR*: There was a problem copying sources to ${MOLA_DEBSRC_DIR}... aborting script."
	exit 1
fi

cd ${MOLA_DEBSRC_DIR}

# Copy debian directory:
mkdir debian
cp -r ${MOLA_EXTERN_DEBIAN_DIR}/* debian

# Export signing pub key:
mkdir debian/upstream/ || true
gpg --export --export-options export-minimal --armor > debian/upstream/signing-key.asc

# Parse debian/ control.in --> control
mv debian/control.in debian/control
sed -i "s/@MOLA_VER_MM@/${MOLA_VER_MM}/g" debian/control

# Parse libmolaM.M.install.in:
mv debian/libmolaM.M.install.in debian/libmola${MOLA_VER_MM}.install

# Strip my custom files...
rm debian/*.new || true

# Prepare install files:
# For each MOLA module, create its "<lib>.install" file:
# TODO !!
#cd libs
#LST_LIBS=$(ls -d */);   # List only directories
#for lib in $LST_LIBS;
#do
#	lib=${lib%/}  # Remove the trailing "/"
#	echo "usr/lib/libmola-${lib}.so.${MOLA_VER_MM}"   > ../debian/libmola-${lib}${MOLA_VER_MM}.install
#	echo "usr/lib/libmola-${lib}.so.${MOLA_VER_MMP}" >> ../debian/libmola-${lib}${MOLA_VER_MM}.install
#done
#cd .. # Back to MOLA root

# Figure out the next Debian version number:
echo "Detecting next Debian version number..."

CHANGELOG_UPSTREAM_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*.*snapshot.*\)-.*/\1/p' )
CHANGELOG_LAST_DEBIAN_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*\).*-\([0-9]*\).*/\2/p' )

echo " -> PREVIOUS UPSTREAM: $CHANGELOG_UPSTREAM_VER -> New: ${MOLA_VERSION_STR}"
echo " -> PREVIOUS DEBIAN VERSION: $CHANGELOG_LAST_DEBIAN_VER"

# If we have the same upstream versions, increase the Debian version, otherwise create a new entry:
if [ "$CHANGELOG_UPSTREAM_VER" = "$MOLA_VERSION_STR" ];
then
	NEW_DEBIAN_VER=$[$CHANGELOG_LAST_DEBIAN_VER + 1]
	echo "Changing to a new Debian version: ${MOLA_VERSION_STR}-${NEW_DEBIAN_VER}"
	DEBCHANGE_CMD="--newversion ${MOLA_VERSION_STR}-${NEW_DEBIAN_VER}"
else
	DEBCHANGE_CMD="--newversion ${MOLA_VERSION_STR}-1"
fi

echo "Adding a new entry to debian/changelog..."

DEBEMAIL="José Luis Blanco Claraco <joseluisblancoc@gmail.com>" debchange $DEBCHANGE_CMD -b --distribution unstable --force-distribution New version of upstream sources.

set +x

echo "==================================================================="
echo "Now, you can build the source Deb package with 'debuild -S -sa -d'"
echo "==================================================================="

cd ..
ls -lh

exit 0
