# Simple makefile to automate common tasks:
# This Makefile is *NOT* the build makefile, just a helper script!

.DEFAULT_GOAL := help
.PHONY: clean build

help:
	@echo "Use: make TARGET. Available targets:"
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | egrep -v -e '^[^[:alnum:]]' -e '^$@$$'

# Get the latest version:
git_update:
	git pull
	git pull  # Repeat (required sometimes due to network or lock file errors)
	git submodule update --init --recursive
	git submodule sync


cmake_init:
	cmake -S. -Bbuild

build:
	cmake --build build --parallel

clean:
	rm -fr build || true

