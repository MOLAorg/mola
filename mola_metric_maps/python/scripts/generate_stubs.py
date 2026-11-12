#!/usr/bin/env python3
"""Regenerates the type stubs of the extension module, `mola_metric_maps/_core.pyi`.

Run it after changing the C++ bindings, with the built `mola_metric_maps`
package and the `mrpt` and `mp2p_icp` Python modules importable (e.g. after
`source install/setup.bash`):

    pip install pybind11-stubgen==3.0.0
    python3 mola_metric_maps/python/scripts/generate_stubs.py

Stubs depend on the pybind11 version: generate them with the one of the
mrpt3 wheel (mrpt.sdk.PYBIND11_VERSION), e.g. with the mola-metric-maps wheel
installed, not with an older system one.
"""

import importlib
import os
import re
import subprocess
import sys
import tempfile

PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mola_metric_maps")


def use_public_names(text: str) -> str:
    """'mrpt.X._bindings.Name' into 'mrpt.X.Name', and 'mp2p_icp._core.Name'
    into 'mp2p_icp.Name', if the package exports that name."""

    def public(module: str, name: str, original: str) -> str:
        mod = importlib.import_module(module)
        return f"{module}.{name}" if hasattr(mod, name) else original

    text = re.sub(
        r"\bmrpt\.(\w+)\._bindings\.(\w+)\b",
        lambda m: public(f"mrpt.{m.group(1)}", m.group(2), m.group(0)),
        text,
    )
    text = re.sub(
        r"\bmp2p_icp\._core\.(\w+)\b",
        lambda m: public("mp2p_icp", m.group(1), m.group(0)),
        text,
    )
    for private in set(
        re.findall(r"^import ((?:mrpt\.\w+\._bindings)|mp2p_icp\._core)$", text, re.M)
    ):
        if f"{private}." not in text:
            public_mod = private.rsplit(".", 1)[0]
            text = re.sub(
                rf"^import {re.escape(private)}$", f"import {public_mod}", text, flags=re.M
            )
    return text


def main() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(
            [
                sys.executable,
                "-m",
                "pybind11_stubgen",
                "--numpy-array-remove-parameters",
                "-o",
                tmp,
                "mola_metric_maps._core",
            ],
            check=True,
            env=dict(os.environ, PYTHONWARNINGS="ignore"),
        )
        with open(os.path.join(tmp, "mola_metric_maps", "_core.pyi"), encoding="utf-8") as f:
            text = use_public_names(f.read())
    with open(os.path.join(PKG_DIR, "_core.pyi"), "w", encoding="utf-8") as f:
        f.write(text)
    open(os.path.join(PKG_DIR, "py.typed"), "w").close()
    print("Updated:", os.path.normpath(os.path.join(PKG_DIR, "_core.pyi")))


if __name__ == "__main__":
    main()
