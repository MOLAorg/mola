[![CircleCI](https://circleci.com/gh/MOLAorg/mola.svg?style=svg)](https://circleci.com/gh/MOLAorg/mola)

# mola
A Modular Optimization framework for Localization and mApping (MOLA)

This repository holds the MOLA git [superproject](https://en.wikibooks.org/wiki/Git/Submodules_and_Superprojects).

Refer to the [official documentation](https://docs.mola-slam.org/latest/) for instructions, demos, API reference, etc.

## Building
### Getting the sources
```
git clone --recurse-submodules https://github.com/MOLAorg/mola.git
```

### Requisites
* CMake >=3.4
* A C++17 compiler. Either gcc-7, clang-4 or MSVC 2017 are good.
* Library dependencies: For Ubuntu, install them all with:
```
# MRPT >1.9.9, for now from this PPA (or build from sources if preferred):
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt update
sudo apt install libmrpt-dev mrpt-apps

# GTSAM >=4.0.0, from this PPA, or build it from sources:
sudo add-apt-repository ppa:joseluisblancoc/gtsam-develop
sudo apt update
sudo apt install libgtsam-dev

# Boost, yaml-cpp, etc:
sudo apt install libboost-serialization-dev libboost-system-dev \
  libboost-filesystem-dev libboost-thread-dev libboost-program-options-dev \
  libboost-date-time-dev libboost-timer-dev libboost-chrono-dev \
  libboost-regex-dev
sudo apt install libyaml-cpp-dev
```

### Build
Classic cmake stuff:
```
cmake -H. -Bbuild
cmake --build .
```

## Run demos

Read [the documentation](https://docs.mola-slam.org/latest/demos.html).

## About the directory structure
Directories layout is as follows:
* `cmake-modules`: Extra CMake scripts
* `demos`: Example YAML files for `mola-launcher`
* `doc`: Documentation and placeholder for Doxygen docs.
* `externals`: All external depedencies

## How to generate the docs

### Requisities
```
sudo apt install python3-pip
sudo -H pip3 install sphinx_rtd_theme
```

### Generate docs

```
cd mola/..
mkdir mola-www
sphinx-build -b html mola/docs/source/ mola-www/
```

## License
MOLA is released under the GNU GPL v3 license, except noted otherwise in each individual module. Other options available upon request.
