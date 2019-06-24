[![CircleCI](https://img.shields.io/circleci/build/gh/MOLAorg/mola/master.svg)](https://circleci.com/gh/MOLAorg/mola) [![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/latest/)


# mola
A Modular Optimization framework for Localization and mApping (MOLA)

This repository holds the MOLA git [superproject](https://en.wikibooks.org/wiki/Git/Submodules_and_Superprojects).

Refer to the [official documentation](https://docs.mola-slam.org/latest/) for
build instructions, [demos](https://docs.mola-slam.org/latest/demos.html), API reference, etc.

## Building
Clone with:

```
git clone --recurse-submodules https://github.com/MOLAorg/mola.git
```

Follow [these instructions](https://docs.mola-slam.org/latest/building.html) (in RST format [here](docs/source/building.rst)).

## About the directory structure
Directories layout is as follows:
* `demos`: Example YAML files for `mola-launcher`
* `docs`: Documentation and placeholder for Doxygen docs.
* `externals`: All external depedencies
* `modules`: All MOLA module projects.


## License
MOLA is released under the GNU GPL v3 license, except noted otherwise in each individual module. Other options available upon request.
