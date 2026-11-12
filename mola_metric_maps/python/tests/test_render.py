"""Off-screen rendering of maps of each class, with mp2p_icp.render(). Skipped
if off-screen OpenGL rendering is not available."""

from __future__ import annotations

import numpy as np

import mp2p_icp

from conftest import filled_map


def test_render_layer(map_name, scene, render_or_skip):
    m = filled_map(map_name, scene)
    mm = mp2p_icp.MetricMap()
    mm.layers["layer"] = m
    img = render_or_skip(mm, 160, 120, background=(1, 1, 1))
    assert img.shape == (120, 160, 3)
    assert np.any(img < 200, axis=2).mean() > 0.005
