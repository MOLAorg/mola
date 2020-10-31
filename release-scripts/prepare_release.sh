#!/bin/bash
# Export sources from a git tree and prepare it for a public release.
# JLBC, 2008-2020

set -e  # exit on error

# Checks
# --------------------------------
VERSION_FILE="modules/mola-common/mola-version.cmake"
if [ -f ${VERSION_FILE} ];
then
	if [ -z ${MOLA_VERSION_STR+x} ];
	then
		# MOLA_VERSION_STR is not set by caller: load it
    MOLA_VERSION_MAJOR=$(grep MOLA_VERSION_NUMBER_MAJOR ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
    MOLA_VERSION_MINOR=$(grep MOLA_VERSION_NUMBER_MINOR ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
    MOLA_VERSION_PATCH=$(grep MOLA_VERSION_NUMBER_PATCH ${VERSION_FILE} | cut -f 2 -d " " | cut -f 1 -d ")")
		MOLA_VERSION_STR="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}.${MOLA_VERSION_PATCH}"
	fi
	MOLA_VERSION_MAJOR=${MOLA_VERSION_STR:0:1}
	MOLA_VERSION_MINOR=${MOLA_VERSION_STR:2:1}
	MOLA_VERSION_PATCH=${MOLA_VERSION_STR:4:1}
	MOLA_VER_MM="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}"
	MOLA_VER_MMP="${MOLA_VERSION_MAJOR}.${MOLA_VERSION_MINOR}.${MOLA_VERSION_PATCH}"
else
	echo "ERROR: Run this script from the MOLA root directory."
	exit 1
fi

MOLASRC=`pwd`
OUT_RELEASES_DIR="$HOME/mola_release"

OUT_DIR=$OUT_RELEASES_DIR/mola-${MOLA_VERSION_STR}

echo "=========== Generating MOLA release ${MOLA_VER_MMP} =================="
echo "MOLA_VERSION_STR   : ${MOLA_VERSION_STR}"
echo "OUT_DIR            : ${OUT_DIR}"
echo "============================================================"
echo

# Prepare output directory:
rm -fR $OUT_RELEASES_DIR  || true
mkdir -p ${OUT_DIR}

# Export / copy sources to target dir:
if [ -d "$MOLASRC/.git" ];
then
	echo "# Exporting git source tree to ${OUT_DIR}"
	git archive --format=tar HEAD | tar -x -C ${OUT_DIR}

	# Include external submodules:
	EXTERNAL_MODS=$(git submodule | cut -d ' ' -f 3)
	for MOD in $EXTERNAL_MODS;
	do
		echo "...exporting git submodule: ${MOD}"
		cd ${MOLASRC}/$MOD
		git archive --format=tar HEAD | tar -x -C ${OUT_DIR}/$MOD
	done

	cd ${MOLASRC}

	# Remove VCS control files:
	find ${OUT_DIR} -name '.git*' | xargs rm -fr

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)
else
	echo "# Copying sources to ${OUT_DIR}"
	cp -R . ${OUT_DIR}

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(date +%s)
fi

# See https://reproducible-builds.org/specs/source-date-epoch/
echo $SOURCE_DATE_EPOCH > ${OUT_DIR}/SOURCE_DATE_EPOCH

cd ${OUT_DIR}

# Dont include Debian files in releases:
rm -fR release-scripts

# Orig tarball:
cd ..
echo "# Creating orig tarball: mola-${MOLA_VERSION_STR}.tar.gz"
tar czf mola-${MOLA_VERSION_STR}.tar.gz mola-${MOLA_VERSION_STR}

# GPG signature:
gpg --armor --detach-sign mola-${MOLA_VERSION_STR}.tar.gz
