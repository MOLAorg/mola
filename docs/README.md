Root directory for MOLA Sphinx documentation.

# Requisites

```
# Install Sphinx:

# Install dependencies:
sudo -H pip install sphinx_rtd_theme
```


# How to generate

## CMake method

Once you made `cd build && cmake ..` the docs can be generated invoking the target `build_docs`:

```
make build_docs
```

## Manual method

From the MOLA source tree: 

```
cd docs
make
```


