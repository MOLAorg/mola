
.DEFAULT_GOAL := all

# Replace this path with your local build of doxyrest:
export PYTHONPATH := $(HOME)/code/doxyrest_b/doxyrest/sphinx/:$(PYTHONPATH)
export PATH := $(HOME)/code/doxyrest_b/build/doxyrest/bin/Release:$(PATH)


.PHONY: clean
clean:
	rm -fr html
	rm -fr \
		source/xml-dir \
		source/class_* \
		source/enum_* \
		source/namespace_* \
		source/group_* \
		source/struct_* \
		source/global.rst source/doxygen-index.rst \
		2> /dev/null || true

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
