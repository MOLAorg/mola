#!/bin/bash

if [ -z "$1" ]; then
	echo "Error: Usage is $0 <GIT_COMMAND>"
	exit 1	
fi

CWD=$(pwd);
ls -1d */.git | xargs -I FIL dirname FIL | while read line; do echo "Processing: $line"; cd $CWD/$line && git "$@"; done;


