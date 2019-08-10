Root directory for MOLA Sphinx documentation.

# Requisites

```
# Install Sphinx & dependencies:
sudo -H pip install sphinx_rtd_theme
```

Install doxyrest: [instructions](https://github.com/vovkos/doxyrest_b/blob/master/README.rst).


# How to generate docs

From the MOLA source tree:

```
cd docs
make
```

You may need to either modify PATH and PYTHONPATH in your env, or edit the hard-coded paths in mola/docs/Makefile

