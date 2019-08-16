
.DEFAULT_GOAL := all

# Replace this path with your local build of doxyrest:
export PYTHONPATH := $(HOME)/code/doxyrest_b/doxyrest/sphinx/:$(PYTHONPATH)
export PATH := $(HOME)/code/doxyrest_b/build/doxyrest/bin/Release:$(PATH)


.PHONY: clean
clean:
	rm -fr html
	rm -fr source/xml-dir

.PHONY: all
all: 
	make html

html-and-view: 
	make html
	xdg-open html/index.html

html:
	# 1) Build Doxygen
	cd source && doxygen
	# 2) Doxygen XML -> RST
	cd source && doxyrest -c doxyrest-config.lua
	# 3) RST -> HTML with Sphinx
	sphinx-build -b html source/ html/

