#!/bin/bash 

CWD=$(pwd);
ls -1d */.git | xargs -I FIL dirname FIL | while read line; do echo "Processing: $line"; cd $CWD/$line && git pull; done;


