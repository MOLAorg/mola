#!/bin/bash

NUM_PARALLEL_THREADS=8

CWD=$(pwd);
ls -1d */.git | xargs -I FIL dirname FIL | \
while read line; do \
 ((i=i%NUM_PARALLEL_THREADS)); ((i++==0)) && wait
 echo "[thread $i] Processing: $line";\
 cd $CWD/$line; \
 DEFAULT_BRANCH=$(basename $(git symbolic-ref --short refs/remotes/origin/HEAD)); \
 echo ">> Detected default branch for '$line': '$DEFAULT_BRANCH'"; \
 git checkout $DEFAULT_BRANCH; \
 git pull; \
done;

# Wait for all children processes to end:
wait

