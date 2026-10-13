# mola_relocalization
C++ library with algorithms for relocalization, global localization, or pose estimation given a large initial uncertainty.

Note that particle filtering is implemented in its own repository under [mrpt_navigation](https://github.com/mrpt-ros-pkg/mrpt_navigation).

## Method #1: mola::RelocalizationICP_SE2

Takes a global and a local metric map, a SE(2) ROI, and tries to match
the local map in the global map by running ICP from all initial guesses
defined by a regular SE(2) lattice, returning the result as a SE(3) hashed
lattice.

This method is based on [mp2p_icp ICP pipelines](https://docs.mola-slam.org/latest/module-mp2p-icp.html).

Each ICP run whose quality reaches `icp_minimum_quality` is returned in
`Output::candidates`, best first. `mola::select_relocalization_candidate()`
groups them into modes (nearby poses) and tells whether the best one is
unambiguous, i.e. clearly better than any other distinct pose. Repetitive
scenes (e.g. identical crop rows) give several modes of similar quality.

## Method #2: mola::RelocalizationLikelihood_SE2

Takes a global metric map, an observation, and a SE(2) ROI, and evaluates
the likelihood of the observation in a regular SE(2) lattice.

This is based on mrpt maps observationLikelihood() evaluation, so the main
parameters that determine the way likelihood is computed must be defined
on before hand within each of the metric map layers of the input reference
map.
At present this algorithm is useful for these sensor/map types:
- observations: pointclouds/2d_scan, reference_map: 2D gridmap
- observations: pointclouds, reference_map: pointclouds

Example result for the `mola::RelocalizationLikelihood_SE2` method (from a unit test, see [the code](https://github.com/MOLAorg/mola/blob/develop/mola_relocalization/tests/test-relocalization-se2-kitti.cpp) for details):

![mola_relocalize_figs](https://github.com/MOLAorg/mola/assets/5497818/6622739f-95ca-4e39-a770-d5f15c01adb3)

**Figure:** (top-left) Reference map. (bottom-left) Query map (actually, a decimated version is used internally). (top-right) Visualization of a slice of the returned likelihood field over a SE(2) ROI. The "slice" is for orientation (phi) equal to 0 (close to the actual pose transformation between the two maps). (bottom-right) The same likelihood slice, in real (x,y) coordinates (meters). The clear peak reveals, approximately, the location of the sought SE(2) transformation between the maps. Further refining is possible using [ICP](https://github.com/MOLAorg/mp2p_icp) using this as initial guess.


## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the GNU GPL v3 license. Other options available upon request.
