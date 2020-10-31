#!/bin/bash
find . -name '*.changes' | xargs -I FIL dput ppa:joseluisblancoc/mola FIL
