# mola
A Modular Optimization framework for Localization and mApping (MOLA)

(videos)

## Building
### Getting the sources
```
git clone --recurse-submodules git@github.com:MOLA/mola.git
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
mkdir build && cd build
cmake ..
cmake --build .
```

## Run demos
Write me!

## About the directory structure
Directories layout is as follows:
* `cmake-modules`: Extra CMake scripts
* `demos`: Example YAML files for `mola-launcher`
* `doc`: Documentation and placeholder for Doxygen docs.
* `externals`: All external depedencies

## License
MOLA is released under the GNU GPL v3 license. Other options available upon request.
