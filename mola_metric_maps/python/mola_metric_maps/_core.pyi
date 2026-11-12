"""
Python bindings of the mola_metric_maps C++ library (use the `mola_metric_maps` package)
"""
from __future__ import annotations
import mrpt.config
import mrpt.img
import mrpt.maps
import mrpt.math
import mrpt.obs
import mrpt.poses
import mrpt.serialization
import numpy
import typing
__all__: list[str] = ['HAS_INCREMENTAL_KDTREE', 'HAS_INCREMENTAL_KDTREE_BAKE', 'HashedVoxelPointCloud', 'IncrementalPointCloud', 'KeyframePointCloudMap', 'NDT', 'OccGrid', 'PointIndexType', 'SparseTreesPointCloud', 'SparseVoxelPointCloud', 'TSDF', 'ViewDirectionFilter']
class ViewDirectionFilter:
    """
    How the cov-to-cov matching of KeyframePointCloudMap and IncrementalPointCloud uses the per-point view directions (view_x/y/z fields) to reject pairs.
    
    Members:
    
      None_ : No filtering.
    
      MaxAngle : Reject pairs whose view directions are more than max_view_angle_deg apart.
    
      SurfaceSide : Reject pairs seeing a flat surface from opposite sides.
    """
    MaxAngle: typing.ClassVar[ViewDirectionFilter]
    None_: typing.ClassVar[ViewDirectionFilter]
    SurfaceSide: typing.ClassVar[ViewDirectionFilter]
    __members__: typing.ClassVar[dict[str, ViewDirectionFilter]]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class PointIndexType:
    """
    Spatial index of IncrementalPointCloud.
    
    Members:
    
      KDTree : Incremental, self-balancing k-d tree (nanoflann).
    
      VoxelHash : Hashed grid of voxels, with O(1) insertion and removal.
    """
    KDTree: typing.ClassVar[PointIndexType]
    VoxelHash: typing.ClassVar[PointIndexType]
    __members__: typing.ClassVar[dict[str, PointIndexType]]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class HashedVoxelPointCloud(mrpt.maps.CMetricMap):
    """
    A point cloud stored as a sparse hash map of cubic voxels, each with up to 32 points (mola::HashedVoxelPointCloud).
    """
    class TCreationOptions(mrpt.config.CLoadableOptions):
        """
        Options that define the map structure.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def voxel_size(self) -> float:
            """
            Voxel size [m].
            """
        @voxel_size.setter
        def voxel_size(self, arg0: float) -> None:
            ...
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def max_points_per_voxel(self) -> int:
            """
            Maximum number of points per voxel (0: no limit other than 32).
            """
        @max_points_per_voxel.setter
        def max_points_per_voxel(self, arg0: int) -> None:
            ...
        @property
        def min_distance_between_points(self) -> float:
            """
            If not 0, points closer than this [m] to another one in its voxel are not inserted.
            """
        @min_distance_between_points.setter
        def min_distance_between_points(self, arg0: float) -> None:
            ...
        @property
        def remove_voxels_farther_than(self) -> float:
            """
            If not 0, voxels farther than this distance [m] from the robot are removed on each insertion.
            """
        @remove_voxels_farther_than.setter
        def remove_voxels_farther_than(self, arg0: float) -> None:
            ...
    class TLikelihoodOptions(mrpt.config.CLoadableOptions):
        """
        Options for computing the likelihood of observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def decimation(self) -> int:
            """
            Use one out of N points.
            """
        @decimation.setter
        def decimation(self, arg0: int) -> None:
            ...
        @property
        def max_corr_distance(self) -> float:
            """
            Maximum distance [m] of a point to the map, to bound its likelihood.
            """
        @max_corr_distance.setter
        def max_corr_distance(self, arg0: float) -> None:
            ...
        @property
        def sigma_dist(self) -> float:
            """
            Divisor of the squared point-to-map distance in the log-likelihood [m^2].
            """
        @sigma_dist.setter
        def sigma_dist(self, arg0: float) -> None:
            ...
    class TRenderOptions(mrpt.config.CLoadableOptions):
        """
        Options for rendering.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def color(self) -> mrpt.img.TColorf:
            """
            Color of points (mrpt.img.TColorf), if colormap is cmNONE.
            """
        @color.setter
        def color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def colormap(self) -> mrpt.img.TColormap:
            """
            Colormap (mrpt.img.TColormap).
            """
        @colormap.setter
        def colormap(self, arg0: mrpt.img.TColormap) -> None:
            ...
        @property
        def point_size(self) -> float:
            """
            Point size [pixels].
            """
        @point_size.setter
        def point_size(self, arg0: float) -> None:
            ...
        @property
        def recolorByPointField(self) -> str:
            """
            Point field used for the colormap.
            """
        @recolorByPointField.setter
        def recolorByPointField(self, arg0: str) -> None:
            ...
    def __copy__(self) -> HashedVoxelPointCloud:
        """
        A deep copy of the map.
        """
    def __deepcopy__(self, memo: typing.Any) -> HashedVoxelPointCloud:
        """
        A deep copy of the map.
        """
    def __getstate__(self) -> bytes:
        ...
    def __init__(self, voxel_size: float = 1.0) -> None:
        """
        Creates an empty map, with voxels of this size [m].
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def getPointsAsNumpy(self) -> numpy.ndarray:
        """
        All points in the map, as an (N,3) float32 array.
        """
    def getVoxelsAsNumpy(self) -> dict:
        """
        The voxels, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) float32) and 'num_points' (uint32).
        """
    def insertPoint(self, point: typing.Any) -> None:
        """
        Inserts one point, in map coordinates (mrpt.math.TPoint3D or [x, y, z]).
        """
    def insertPointCloud(self, points: typing.Any, robotPose: typing.Any = None) -> bool:
        """
        Inserts a point cloud, as insertObservation() with an mrpt.obs.CObservationPointCloud. `points` (an (N,3) array, a dict {field name: 1-D array} as in mp2p_icp.layer_from_numpy(), or an mrpt.maps.CPointsMap, which is copied) are given in the robot frame, and the robot is at `robotPose` in the map: None (identity), mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 matrix or [x, y, z, yaw, pitch, roll]. Maps that trace rays (TSDF, NDT) take the robot position as their origin. Insertion options and map-specific behavior apply, e.g. removal of far contents. Returns True if the map was updated.
        """
    def nn_multiple_search(self, query: typing.Any, N: int) -> tuple:
        """
        The N closest points to a 3D query point, sorted by distance. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_radius_search(self, query: typing.Any, search_radius_sqr: float, maxPoints: int = 0) -> tuple:
        """
        The points within a distance of a 3D query point. The radius is given squared; maxPoints=0 means no limit. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_single_search(self, query: typing.Any) -> typing.Any:
        """
        Closest point to a 3D query point (mrpt.math.TPoint3D or [x, y, z]). Returns (index or ID, point, squared distance), or None if none was found.
        """
    def nn_single_search_batch(self, queries: typing.Any) -> dict:
        """
        Closest point to each row of an (N,3) array of query points. Returns a dict with 'found' (bool), 'indices' (uint64), 'points' ((N,3) float32, NaN if not found) and 'dist_sqr' (float32, inf if not found). Releases the GIL.
        """
    def optionsByName(self) -> dict[str, mrpt.config.CLoadableOptions]:
        """
        A dict {name: options} with all the option groups of the map, e.g. 'insertionOptions'. They are the live objects, except that 'creationOptions' must not be modified in place: use trySetCreationOptions().
        """
    def saveToTextFile(self, file: str) -> bool:
        """
        Saves all points to a text file, one 'x y z' per line. Returns False on error.
        """
    def setVoxelProperties(self, voxel_size: float) -> None:
        """
        Changes the voxel size [m], CLEARING the map.
        """
    @typing.overload
    def trySetCreationOptions(self, options: HashedVoxelPointCloud.TCreationOptions) -> bool:
        """
        Applies new creation options. Returns False, leaving the map untouched, if they cannot be applied without discarding its contents (e.g. a different voxel size).
        """
    @typing.overload
    def trySetCreationOptions(self, cfg: mrpt.config.CConfigFileBase, section: str) -> bool:
        """
        Like trySetCreationOptions(options), reading them from a section of a configuration file (mrpt.config.CConfigFileBase).
        """
    def voxelCount(self) -> int:
        """
        Number of voxels in the map.
        """
    @property
    def creationOptions(self) -> HashedVoxelPointCloud.TCreationOptions:
        """
        Options that define the map structure (TCreationOptions). Reading it returns a COPY: modify it and assign it back, which applies it as trySetCreationOptions() does, raising ValueError if it is refused.
        """
    @creationOptions.setter
    def creationOptions(self, arg1: HashedVoxelPointCloud.TCreationOptions) -> None:
        ...
    @property
    def insertionOptions(self) -> HashedVoxelPointCloud.TInsertionOptions:
        """
        Insertion options (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: HashedVoxelPointCloud.TInsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> HashedVoxelPointCloud.TLikelihoodOptions:
        """
        Likelihood options (TLikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: HashedVoxelPointCloud.TLikelihoodOptions) -> None:
        ...
    @property
    def renderOptions(self) -> HashedVoxelPointCloud.TRenderOptions:
        """
        Render options (TRenderOptions).
        """
    @renderOptions.setter
    def renderOptions(self, arg0: HashedVoxelPointCloud.TRenderOptions) -> None:
        ...
class SparseVoxelPointCloud(mrpt.maps.CMetricMap):
    """
    A point cloud in a two-level voxel map: a sparse map of dense blocks of 32x32x32 voxels, each with up to 16 points and their mean (mola::SparseVoxelPointCloud).
    """
    class TCreationOptions(mrpt.config.CLoadableOptions):
        """
        Options that define the map structure.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def voxel_size(self) -> float:
            """
            Voxel size [m].
            """
        @voxel_size.setter
        def voxel_size(self, arg0: float) -> None:
            ...
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def max_points_per_voxel(self) -> int:
            """
            Maximum number of points per voxel (0: no limit other than 16).
            """
        @max_points_per_voxel.setter
        def max_points_per_voxel(self, arg0: int) -> None:
            ...
    class TLikelihoodOptions(mrpt.config.CLoadableOptions):
        """
        Options for computing the likelihood of observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def decimation(self) -> int:
            """
            Use one out of N points.
            """
        @decimation.setter
        def decimation(self, arg0: int) -> None:
            ...
        @property
        def match_mean(self) -> bool:
            """
            Match against the voxel means instead of their points, in the likelihood and the nn_*() queries.
            """
        @match_mean.setter
        def match_mean(self, arg0: bool) -> None:
            ...
        @property
        def max_corr_distance(self) -> float:
            """
            Maximum distance [m] of a point to the map, to bound its likelihood.
            """
        @max_corr_distance.setter
        def max_corr_distance(self, arg0: float) -> None:
            ...
        @property
        def sigma_dist(self) -> float:
            """
            Divisor of the squared point-to-map distance in the log-likelihood [m^2].
            """
        @sigma_dist.setter
        def sigma_dist(self, arg0: float) -> None:
            ...
    class TRenderOptions(mrpt.config.CLoadableOptions):
        """
        Options for rendering.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def color(self) -> mrpt.img.TColorf:
            """
            Color of points (mrpt.img.TColorf), if colormap is cmNONE.
            """
        @color.setter
        def color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def colormap(self) -> mrpt.img.TColormap:
            """
            Colormap (mrpt.img.TColormap).
            """
        @colormap.setter
        def colormap(self, arg0: mrpt.img.TColormap) -> None:
            ...
        @property
        def point_size(self) -> float:
            """
            Point size [pixels].
            """
        @point_size.setter
        def point_size(self, arg0: float) -> None:
            ...
        @property
        def recolorByPointField(self) -> str:
            """
            Point field used for the colormap.
            """
        @recolorByPointField.setter
        def recolorByPointField(self, arg0: str) -> None:
            ...
        @property
        def show_inner_grid_boxes(self) -> bool:
            """
            Render the boxes of the dense blocks.
            """
        @show_inner_grid_boxes.setter
        def show_inner_grid_boxes(self, arg0: bool) -> None:
            ...
        @property
        def show_mean_only(self) -> bool:
            """
            Render only the mean point of each voxel.
            """
        @show_mean_only.setter
        def show_mean_only(self, arg0: bool) -> None:
            ...
    def __copy__(self) -> SparseVoxelPointCloud:
        """
        A deep copy of the map.
        """
    def __deepcopy__(self, memo: typing.Any) -> SparseVoxelPointCloud:
        """
        A deep copy of the map.
        """
    def __getstate__(self) -> bytes:
        ...
    def __init__(self, voxel_size: float = 0.20000000298023224) -> None:
        """
        Creates an empty map, with voxels of this size [m].
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def getPointsAsNumpy(self) -> numpy.ndarray:
        """
        All points in the map, as an (N,3) float32 array.
        """
    def getVoxelsAsNumpy(self) -> dict:
        """
        The non-empty voxels, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) float32), 'num_points' (uint32) and 'means' ((M,3) float32, mean of their points).
        """
    def insertPoint(self, point: typing.Any) -> None:
        """
        Inserts one point, in map coordinates (mrpt.math.TPoint3D or [x, y, z]).
        """
    def insertPointCloud(self, points: typing.Any, robotPose: typing.Any = None) -> bool:
        """
        Inserts a point cloud, as insertObservation() with an mrpt.obs.CObservationPointCloud. `points` (an (N,3) array, a dict {field name: 1-D array} as in mp2p_icp.layer_from_numpy(), or an mrpt.maps.CPointsMap, which is copied) are given in the robot frame, and the robot is at `robotPose` in the map: None (identity), mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 matrix or [x, y, z, yaw, pitch, roll]. Maps that trace rays (TSDF, NDT) take the robot position as their origin. Insertion options and map-specific behavior apply, e.g. removal of far contents. Returns True if the map was updated.
        """
    def nn_multiple_search(self, query: typing.Any, N: int) -> tuple:
        """
        The N closest points to a 3D query point, sorted by distance. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_radius_search(self, query: typing.Any, search_radius_sqr: float, maxPoints: int = 0) -> tuple:
        """
        The points within a distance of a 3D query point. The radius is given squared; maxPoints=0 means no limit. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_single_search(self, query: typing.Any) -> typing.Any:
        """
        Closest point to a 3D query point (mrpt.math.TPoint3D or [x, y, z]). Returns (index or ID, point, squared distance), or None if none was found.
        """
    def nn_single_search_batch(self, queries: typing.Any) -> dict:
        """
        Closest point to each row of an (N,3) array of query points. Returns a dict with 'found' (bool), 'indices' (uint64), 'points' ((N,3) float32, NaN if not found) and 'dist_sqr' (float32, inf if not found). Releases the GIL.
        """
    def optionsByName(self) -> dict[str, mrpt.config.CLoadableOptions]:
        """
        A dict {name: options} with all the option groups of the map, e.g. 'insertionOptions'. They are the live objects, except that 'creationOptions' must not be modified in place: use trySetCreationOptions().
        """
    def saveToTextFile(self, file: str) -> bool:
        """
        Saves all points to a text file, one 'x y z' per line. Returns False on error.
        """
    def setVoxelProperties(self, voxel_size: float) -> None:
        """
        Changes the voxel size [m], CLEARING the map.
        """
    @typing.overload
    def trySetCreationOptions(self, options: SparseVoxelPointCloud.TCreationOptions) -> bool:
        """
        Applies new creation options. Returns False, leaving the map untouched, if they cannot be applied without discarding its contents (e.g. a different voxel size).
        """
    @typing.overload
    def trySetCreationOptions(self, cfg: mrpt.config.CConfigFileBase, section: str) -> bool:
        """
        Like trySetCreationOptions(options), reading them from a section of a configuration file (mrpt.config.CConfigFileBase).
        """
    @property
    def creationOptions(self) -> SparseVoxelPointCloud.TCreationOptions:
        """
        Options that define the map structure (TCreationOptions). Reading it returns a COPY: modify it and assign it back, which applies it as trySetCreationOptions() does, raising ValueError if it is refused.
        """
    @creationOptions.setter
    def creationOptions(self, arg1: SparseVoxelPointCloud.TCreationOptions) -> None:
        ...
    @property
    def insertionOptions(self) -> SparseVoxelPointCloud.TInsertionOptions:
        """
        Insertion options (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: SparseVoxelPointCloud.TInsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> SparseVoxelPointCloud.TLikelihoodOptions:
        """
        Likelihood options (TLikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: SparseVoxelPointCloud.TLikelihoodOptions) -> None:
        ...
    @property
    def renderOptions(self) -> SparseVoxelPointCloud.TRenderOptions:
        """
        Render options (TRenderOptions).
        """
    @renderOptions.setter
    def renderOptions(self, arg0: SparseVoxelPointCloud.TRenderOptions) -> None:
        ...
class SparseTreesPointCloud(mrpt.maps.CMetricMap):
    """
    A point cloud split into a sparse grid of blocks, each a point cloud with its own k-d tree (mola::SparseTreesPointCloud). Experimental.
    """
    class TCreationOptions(mrpt.config.CLoadableOptions):
        """
        Options that define the map structure.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def grid_size(self) -> float:
            """
            Size of the blocks [m].
            """
        @grid_size.setter
        def grid_size(self, arg0: float) -> None:
            ...
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def minimum_points_clearance(self) -> float:
            """
            Points closer than this [m] to an existing one are not inserted.
            """
        @minimum_points_clearance.setter
        def minimum_points_clearance(self, arg0: float) -> None:
            ...
        @property
        def remove_submaps_farther_than(self) -> float:
            """
            If not 0, blocks farther than this distance [m] from the robot are removed on each insertion.
            """
        @remove_submaps_farther_than.setter
        def remove_submaps_farther_than(self, arg0: float) -> None:
            ...
    class TLikelihoodOptions(mrpt.config.CLoadableOptions):
        """
        Options for computing the likelihood of observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def decimation(self) -> int:
            """
            Use one out of N points.
            """
        @decimation.setter
        def decimation(self, arg0: int) -> None:
            ...
        @property
        def max_corr_distance(self) -> float:
            """
            Maximum distance [m] of a point to the map, to bound its likelihood.
            """
        @max_corr_distance.setter
        def max_corr_distance(self, arg0: float) -> None:
            ...
        @property
        def sigma_dist(self) -> float:
            """
            Divisor of the squared point-to-map distance in the log-likelihood [m^2].
            """
        @sigma_dist.setter
        def sigma_dist(self, arg0: float) -> None:
            ...
    class TRenderOptions(mrpt.config.CLoadableOptions):
        """
        Options for rendering.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def color(self) -> mrpt.img.TColorf:
            """
            Color of points (mrpt.img.TColorf), if colormap is cmNONE.
            """
        @color.setter
        def color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def colormap(self) -> mrpt.img.TColormap:
            """
            Colormap (mrpt.img.TColormap).
            """
        @colormap.setter
        def colormap(self, arg0: mrpt.img.TColormap) -> None:
            ...
        @property
        def point_size(self) -> float:
            """
            Point size [pixels].
            """
        @point_size.setter
        def point_size(self, arg0: float) -> None:
            ...
        @property
        def recolorByPointField(self) -> str:
            """
            Point field used for the colormap.
            """
        @recolorByPointField.setter
        def recolorByPointField(self, arg0: str) -> None:
            ...
        @property
        def show_inner_grid_boxes(self) -> bool:
            """
            Render the boxes of the blocks.
            """
        @show_inner_grid_boxes.setter
        def show_inner_grid_boxes(self, arg0: bool) -> None:
            ...
    def __copy__(self) -> SparseTreesPointCloud:
        """
        A deep copy of the map.
        """
    def __deepcopy__(self, memo: typing.Any) -> SparseTreesPointCloud:
        """
        A deep copy of the map.
        """
    def __getstate__(self) -> bytes:
        ...
    def __init__(self, grid_size: float = 10.0) -> None:
        """
        Creates an empty map, with blocks of this size [m].
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def eraseGridsFartherThan(self, point: typing.Any, distance: float) -> None:
        """
        Removes the blocks entirely farther than `distance` [m].
        """
    def getGridsAsNumpy(self) -> dict:
        """
        The blocks, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) float32) and 'num_points' (uint32).
        """
    def getPointsAsNumpy(self) -> numpy.ndarray:
        """
        All points in the map, as an (N,3) float32 array.
        """
    def insertPoint(self, point: typing.Any) -> None:
        """
        Inserts one point, in map coordinates (mrpt.math.TPoint3D or [x, y, z]).
        """
    def insertPointCloud(self, points: typing.Any, robotPose: typing.Any = None) -> bool:
        """
        Inserts a point cloud, as insertObservation() with an mrpt.obs.CObservationPointCloud. `points` (an (N,3) array, a dict {field name: 1-D array} as in mp2p_icp.layer_from_numpy(), or an mrpt.maps.CPointsMap, which is copied) are given in the robot frame, and the robot is at `robotPose` in the map: None (identity), mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 matrix or [x, y, z, yaw, pitch, roll]. Maps that trace rays (TSDF, NDT) take the robot position as their origin. Insertion options and map-specific behavior apply, e.g. removal of far contents. Returns True if the map was updated.
        """
    def nn_multiple_search(self, query: typing.Any, N: int) -> tuple:
        """
        The N closest points to a 3D query point, sorted by distance. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_radius_search(self, query: typing.Any, search_radius_sqr: float, maxPoints: int = 0) -> tuple:
        """
        The points within a distance of a 3D query point. The radius is given squared; maxPoints=0 means no limit. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_single_search(self, query: typing.Any) -> typing.Any:
        """
        Closest point to a 3D query point (mrpt.math.TPoint3D or [x, y, z]). Returns (index or ID, point, squared distance), or None if none was found.
        """
    def nn_single_search_batch(self, queries: typing.Any) -> dict:
        """
        Closest point to each row of an (N,3) array of query points. Returns a dict with 'found' (bool), 'indices' (uint64), 'points' ((N,3) float32, NaN if not found) and 'dist_sqr' (float32, inf if not found). Releases the GIL.
        """
    def optionsByName(self) -> dict[str, mrpt.config.CLoadableOptions]:
        """
        A dict {name: options} with all the option groups of the map, e.g. 'insertionOptions'. They are the live objects, except that 'creationOptions' must not be modified in place: use trySetCreationOptions().
        """
    def saveToTextFile(self, file: str) -> bool:
        """
        Saves all points to a text file, one 'x y z' per line. Returns False on error.
        """
    def setGridProperties(self, grid_size: float) -> None:
        """
        Changes the size of the blocks [m], CLEARING the map.
        """
    @typing.overload
    def trySetCreationOptions(self, options: SparseTreesPointCloud.TCreationOptions) -> bool:
        """
        Applies new creation options. Returns False, leaving the map untouched, if they cannot be applied without discarding its contents (e.g. a different voxel size).
        """
    @typing.overload
    def trySetCreationOptions(self, cfg: mrpt.config.CConfigFileBase, section: str) -> bool:
        """
        Like trySetCreationOptions(options), reading them from a section of a configuration file (mrpt.config.CConfigFileBase).
        """
    @property
    def creationOptions(self) -> SparseTreesPointCloud.TCreationOptions:
        """
        Options that define the map structure (TCreationOptions). Reading it returns a COPY: modify it and assign it back, which applies it as trySetCreationOptions() does, raising ValueError if it is refused.
        """
    @creationOptions.setter
    def creationOptions(self, arg1: SparseTreesPointCloud.TCreationOptions) -> None:
        ...
    @property
    def insertionOptions(self) -> SparseTreesPointCloud.TInsertionOptions:
        """
        Insertion options (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: SparseTreesPointCloud.TInsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> SparseTreesPointCloud.TLikelihoodOptions:
        """
        Likelihood options (TLikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: SparseTreesPointCloud.TLikelihoodOptions) -> None:
        ...
    @property
    def renderOptions(self) -> SparseTreesPointCloud.TRenderOptions:
        """
        Render options (TRenderOptions).
        """
    @renderOptions.setter
    def renderOptions(self, arg0: SparseTreesPointCloud.TRenderOptions) -> None:
        ...
class NDT(mrpt.maps.CMetricMap):
    """
    3D Normal Distributions Transform map: voxels with up to 16 points and the Gaussian fitted to them, which may be a plane (mola::NDT).
    """
    class TCreationOptions(mrpt.config.CLoadableOptions):
        """
        Options that define the map structure.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def voxel_size(self) -> float:
            """
            Voxel size [m].
            """
        @voxel_size.setter
        def voxel_size(self, arg0: float) -> None:
            ...
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def max_eigen_ratio_for_planes(self) -> float:
            """
            A voxel is a plane if its smallest eigenvalue is below this ratio of the second one.
            """
        @max_eigen_ratio_for_planes.setter
        def max_eigen_ratio_for_planes(self, arg0: float) -> None:
            ...
        @property
        def max_points_per_voxel(self) -> int:
            """
            Maximum number of points per voxel (0: no limit other than 16).
            """
        @max_points_per_voxel.setter
        def max_points_per_voxel(self, arg0: int) -> None:
            ...
        @property
        def min_distance_between_points(self) -> float:
            """
            If not 0, points closer than this [m] to another one in its voxel are not inserted.
            """
        @min_distance_between_points.setter
        def min_distance_between_points(self, arg0: float) -> None:
            ...
        @property
        def remove_voxels_farther_than(self) -> float:
            """
            If not 0, voxels farther than this distance [m] from the robot are removed on each insertion.
            """
        @remove_voxels_farther_than.setter
        def remove_voxels_farther_than(self, arg0: float) -> None:
            ...
    class TLikelihoodOptions(mrpt.config.CLoadableOptions):
        """
        Options for computing the likelihood of observations (not used yet).
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def decimation(self) -> int:
            """
            Use one out of N points.
            """
        @decimation.setter
        def decimation(self, arg0: int) -> None:
            ...
        @property
        def max_corr_distance(self) -> float:
            """
            Maximum distance [m] of a point to the map, to bound its likelihood.
            """
        @max_corr_distance.setter
        def max_corr_distance(self, arg0: float) -> None:
            ...
        @property
        def sigma_dist(self) -> float:
            """
            Divisor of the squared point-to-map distance in the log-likelihood [m^2].
            """
        @sigma_dist.setter
        def sigma_dist(self, arg0: float) -> None:
            ...
    class TRenderOptions(mrpt.config.CLoadableOptions):
        """
        Options for rendering.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def normals_color(self) -> mrpt.img.TColorf:
            """
            Color of normals (mrpt.img.TColorf).
            """
        @normals_color.setter
        def normals_color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def normals_visible(self) -> bool:
            """
            Render the plane normals.
            """
        @normals_visible.setter
        def normals_visible(self, arg0: bool) -> None:
            ...
        @property
        def planes_color(self) -> mrpt.img.TColorf:
            """
            Color of planes (mrpt.img.TColorf), if planes_colormap is cmNONE.
            """
        @planes_color.setter
        def planes_color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def planes_colormap(self) -> mrpt.img.TColormap:
            """
            Colormap of planes (mrpt.img.TColormap).
            """
        @planes_colormap.setter
        def planes_colormap(self, arg0: mrpt.img.TColormap) -> None:
            ...
        @property
        def planes_visible(self) -> bool:
            """
            Render the planes.
            """
        @planes_visible.setter
        def planes_visible(self, arg0: bool) -> None:
            ...
        @property
        def point_size(self) -> float:
            """
            Point size [pixels].
            """
        @point_size.setter
        def point_size(self, arg0: float) -> None:
            ...
        @property
        def points_color(self) -> mrpt.img.TColorf:
            """
            Color of points (mrpt.img.TColorf), if points_colormap is cmNONE.
            """
        @points_color.setter
        def points_color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def points_colormap(self) -> mrpt.img.TColormap:
            """
            Colormap of points (mrpt.img.TColormap).
            """
        @points_colormap.setter
        def points_colormap(self, arg0: mrpt.img.TColormap) -> None:
            ...
        @property
        def points_visible(self) -> bool:
            """
            Render the points.
            """
        @points_visible.setter
        def points_visible(self, arg0: bool) -> None:
            ...
        @property
        def recolorByPointField(self) -> str:
            """
            Point field used for the colormap.
            """
        @recolorByPointField.setter
        def recolorByPointField(self, arg0: str) -> None:
            ...
    def __copy__(self) -> NDT:
        """
        A deep copy of the map.
        """
    def __deepcopy__(self, memo: typing.Any) -> NDT:
        """
        A deep copy of the map.
        """
    def __getstate__(self) -> bytes:
        ...
    def __init__(self, voxel_size: float = 5.0) -> None:
        """
        Creates an empty map, with voxels of this size [m].
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def getPointsAsNumpy(self) -> numpy.ndarray:
        """
        All points in the map, as an (N,3) float32 array.
        """
    def getVoxelsAsNumpy(self) -> dict:
        """
        The voxels, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) float32), 'num_points' (uint32), 'has_ndt' (bool: enough points to fit a Gaussian), 'is_plane' (bool), and from the Gaussian, NaN if there is none: 'means' ((M,3) float32), 'eigenvalues' ((M,3) float32, ascending) and 'normals' ((M,3) float32, eigenvector of the smallest eigenvalue, pointing to the sensor).
        """
    def insertPoint(self, point: typing.Any, sensor: typing.Any) -> None:
        """
        Inserts one point, and the sensor position it was seen from (used to orient the plane normals), both in map coordinates (mrpt.math.TPoint3D or [x, y, z]).
        """
    def insertPointCloud(self, points: typing.Any, robotPose: typing.Any = None) -> bool:
        """
        Inserts a point cloud, as insertObservation() with an mrpt.obs.CObservationPointCloud. `points` (an (N,3) array, a dict {field name: 1-D array} as in mp2p_icp.layer_from_numpy(), or an mrpt.maps.CPointsMap, which is copied) are given in the robot frame, and the robot is at `robotPose` in the map: None (identity), mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 matrix or [x, y, z, yaw, pitch, roll]. Maps that trace rays (TSDF, NDT) take the robot position as their origin. Insertion options and map-specific behavior apply, e.g. removal of far contents. Returns True if the map was updated.
        """
    def nn_multiple_search(self, query: typing.Any, N: int) -> tuple:
        """
        The N closest points to a 3D query point, sorted by distance. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_radius_search(self, query: typing.Any, search_radius_sqr: float, maxPoints: int = 0) -> tuple:
        """
        The points within a distance of a 3D query point. The radius is given squared; maxPoints=0 means no limit. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_search_pt2pl(self, point: typing.Any, max_search_distance: float) -> typing.Any:
        """
        Closest plane to a 3D point (mrpt.math.TPoint3D or [x, y, z]), within a search distance. Returns (mp2p_icp.PlanePatch, absolute point-to-plane distance), or None.
        """
    def nn_single_search(self, query: typing.Any) -> typing.Any:
        """
        Closest point to a 3D query point (mrpt.math.TPoint3D or [x, y, z]). Returns (index or ID, point, squared distance), or None if none was found.
        """
    def nn_single_search_batch(self, queries: typing.Any) -> dict:
        """
        Closest point to each row of an (N,3) array of query points. Returns a dict with 'found' (bool), 'indices' (uint64), 'points' ((N,3) float32, NaN if not found) and 'dist_sqr' (float32, inf if not found). Releases the GIL.
        """
    def optionsByName(self) -> dict[str, mrpt.config.CLoadableOptions]:
        """
        A dict {name: options} with all the option groups of the map, e.g. 'insertionOptions'. They are the live objects, except that 'creationOptions' must not be modified in place: use trySetCreationOptions().
        """
    def saveToTextFile(self, file: str) -> bool:
        """
        Saves all points to a text file, one 'x y z' per line. Returns False on error.
        """
    def setVoxelProperties(self, voxel_size: float) -> None:
        """
        Changes the voxel size [m], CLEARING the map.
        """
    @typing.overload
    def trySetCreationOptions(self, options: NDT.TCreationOptions) -> bool:
        """
        Applies new creation options. Returns False, leaving the map untouched, if they cannot be applied without discarding its contents (e.g. a different voxel size).
        """
    @typing.overload
    def trySetCreationOptions(self, cfg: mrpt.config.CConfigFileBase, section: str) -> bool:
        """
        Like trySetCreationOptions(options), reading them from a section of a configuration file (mrpt.config.CConfigFileBase).
        """
    def voxelCount(self) -> int:
        """
        Number of voxels in the map.
        """
    @property
    def creationOptions(self) -> NDT.TCreationOptions:
        """
        Options that define the map structure (TCreationOptions). Reading it returns a COPY: modify it and assign it back, which applies it as trySetCreationOptions() does, raising ValueError if it is refused.
        """
    @creationOptions.setter
    def creationOptions(self, arg1: NDT.TCreationOptions) -> None:
        ...
    @property
    def insertionOptions(self) -> NDT.TInsertionOptions:
        """
        Insertion options (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: NDT.TInsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> NDT.TLikelihoodOptions:
        """
        Likelihood options (TLikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: NDT.TLikelihoodOptions) -> None:
        ...
    @property
    def renderOptions(self) -> NDT.TRenderOptions:
        """
        Render options (TRenderOptions).
        """
    @renderOptions.setter
    def renderOptions(self, arg0: NDT.TRenderOptions) -> None:
        ...
class TSDF(mrpt.maps.CMetricMap):
    """
    Truncated signed distance field, stored as a voxel hash map (mola::TSDF). Each point inserted with its sensor position updates the voxels around its ray.
    """
    class TCreationOptions(mrpt.config.CLoadableOptions):
        """
        Options that define the map structure.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def voxel_size(self) -> float:
            """
            Voxel size [m].
            """
        @voxel_size.setter
        def voxel_size(self, arg0: float) -> None:
            ...
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def bootstrap_field_ready_fraction(self) -> float:
            """
            Fraction of the last scan the field must answer to end the warm-up.
            """
        @bootstrap_field_ready_fraction.setter
        def bootstrap_field_ready_fraction(self, arg0: float) -> None:
            ...
        @property
        def bootstrap_knn(self) -> int:
            """
            Neighbors for the warm-up plane fits.
            """
        @bootstrap_knn.setter
        def bootstrap_knn(self, arg0: int) -> None:
            ...
        @property
        def bootstrap_max_plane_deviation(self) -> float:
            """
            Maximum distance [m] of a neighbor to a warm-up plane (<=0: no limit).
            """
        @bootstrap_max_plane_deviation.setter
        def bootstrap_max_plane_deviation(self, arg0: float) -> None:
            ...
        @property
        def bootstrap_max_scans(self) -> int:
            """
            Maximum length of the warm-up.
            """
        @bootstrap_max_scans.setter
        def bootstrap_max_scans(self, arg0: int) -> None:
            ...
        @property
        def bootstrap_min_scans(self) -> int:
            """
            Scans to insert before the field is tested.
            """
        @bootstrap_min_scans.setter
        def bootstrap_min_scans(self, arg0: int) -> None:
            ...
        @property
        def bootstrap_with_points(self) -> bool:
            """
            Answer queries from the points of the first scans until the field can answer.
            """
        @bootstrap_with_points.setter
        def bootstrap_with_points(self, arg0: bool) -> None:
            ...
        @property
        def max_voxels(self) -> int:
            """
            If not 0, maximum number of voxels in the map.
            """
        @max_voxels.setter
        def max_voxels(self, arg0: int) -> None:
            ...
        @property
        def max_weight(self) -> float:
            """
            Weight cap of a voxel.
            """
        @max_weight.setter
        def max_weight(self, arg0: float) -> None:
            ...
        @property
        def min_weight_for_query(self) -> float:
            """
            Minimum weight of a voxel for it to take part in a query.
            """
        @min_weight_for_query.setter
        def min_weight_for_query(self, arg0: float) -> None:
            ...
        @property
        def ray_tube_voxels(self) -> float:
            """
            Radius, in voxels, of the cylinder around each ray that is updated.
            """
        @ray_tube_voxels.setter
        def ray_tube_voxels(self, arg0: float) -> None:
            ...
        @property
        def remove_voxels_farther_than(self) -> float:
            """
            If not 0, voxels farther than this distance [m] from the sensor are removed.
            """
        @remove_voxels_farther_than.setter
        def remove_voxels_farther_than(self, arg0: float) -> None:
            ...
        @property
        def truncation_distance(self) -> float:
            """
            Half-width [m] of the band updated around surfaces, also the maximum distance at which a query finds a surface. If <=0, truncation_voxels times the voxel size.
            """
        @truncation_distance.setter
        def truncation_distance(self, arg0: float) -> None:
            ...
        @property
        def truncation_voxels(self) -> float:
            """
            Truncation distance in voxels, if truncation_distance <= 0.
            """
        @truncation_voxels.setter
        def truncation_voxels(self, arg0: float) -> None:
            ...
        @property
        def tube_sigma_voxels(self) -> float:
            """
            Sigma, in voxels, of the Gaussian profile across the ray tube (<=0: flat).
            """
        @tube_sigma_voxels.setter
        def tube_sigma_voxels(self, arg0: float) -> None:
            ...
        @property
        def weight_by_incidence(self) -> bool:
            """
            Weight samples by the cosine of their incidence angle.
            """
        @weight_by_incidence.setter
        def weight_by_incidence(self, arg0: bool) -> None:
            ...
        @property
        def weight_by_range(self) -> bool:
            """
            Weight samples by (weight_range_ref/range)^2.
            """
        @weight_by_range.setter
        def weight_by_range(self, arg0: bool) -> None:
            ...
        @property
        def weight_range_ref(self) -> float:
            """
            Reference range [m] for weight_by_range.
            """
        @weight_range_ref.setter
        def weight_range_ref(self, arg0: float) -> None:
            ...
    class TRenderOptions(mrpt.config.CLoadableOptions):
        """
        Options for rendering.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def colormap(self) -> mrpt.img.TColormap:
            """
            Colormap (mrpt.img.TColormap).
            """
        @colormap.setter
        def colormap(self, arg0: mrpt.img.TColormap) -> None:
            ...
        @property
        def point_size(self) -> float:
            """
            Point size [pixels].
            """
        @point_size.setter
        def point_size(self, arg0: float) -> None:
            ...
        @property
        def points_color(self) -> mrpt.img.TColorf:
            """
            Color of the surface (mrpt.img.TColorf), if colormap is cmNONE, and of the mesh.
            """
        @points_color.setter
        def points_color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def recolorize_by(self) -> str:
            """
            Quantity used for the colormap: 'z' or 'weight'.
            """
        @recolorize_by.setter
        def recolorize_by(self, arg0: str) -> None:
            ...
        @property
        def render_as_mesh(self) -> bool:
            """
            Render the surface as a mesh instead of points (slower).
            """
        @render_as_mesh.setter
        def render_as_mesh(self, arg0: bool) -> None:
            ...
    def __copy__(self) -> TSDF:
        """
        A deep copy of the map.
        """
    def __deepcopy__(self, memo: typing.Any) -> TSDF:
        """
        A deep copy of the map.
        """
    def __getstate__(self) -> bytes:
        ...
    def __init__(self, voxel_size: float = 0.30000001192092896) -> None:
        """
        Creates an empty map, with voxels of this size [m].
        """
    def __len__(self) -> int:
        """
        Number of voxels.
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def getPointsAsNumpy(self) -> numpy.ndarray:
        """
        The surface (zero level set of the field, sampled on the voxel lattice edges), as an (N,3) float32 array.
        """
    def getVoxelsAsNumpy(self) -> dict:
        """
        The voxels, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) float32), 'dist' (float32, signed distance [m], positive towards the sensor) and 'weight' (float32).
        """
    def getZeroCrossingsAsNumpy(self) -> tuple:
        """
        Like getPointsAsNumpy(), returning a tuple (points: (N,3) float32, weights: float32 array with the smaller weight of the two voxels of each crossing).
        """
    def insertPoint(self, point: typing.Any, sensor: typing.Any) -> None:
        """
        Fuses one measured point, and the origin of its ray, both in map coordinates (mrpt.math.TPoint3D or [x, y, z]).
        """
    def insertPointCloud(self, points: typing.Any, robotPose: typing.Any = None) -> bool:
        """
        Inserts a point cloud, as insertObservation() with an mrpt.obs.CObservationPointCloud. `points` (an (N,3) array, a dict {field name: 1-D array} as in mp2p_icp.layer_from_numpy(), or an mrpt.maps.CPointsMap, which is copied) are given in the robot frame, and the robot is at `robotPose` in the map: None (identity), mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 matrix or [x, y, z, yaw, pitch, roll]. Maps that trace rays (TSDF, NDT) take the robot position as their origin. Insertion options and map-specific behavior apply, e.g. removal of far contents. Returns True if the map was updated.
        """
    def isBootstrapping(self) -> bool:
        """
        True while queries are answered from the warm-up point cloud, not from the field.
        """
    def lastFieldReadyFraction(self) -> float:
        """
        Fraction of the last inserted scan for which the field answered a query.
        """
    def nn_search_pt2pl(self, point: typing.Any, max_search_distance: float) -> typing.Any:
        """
        Closest plane to a 3D point (mrpt.math.TPoint3D or [x, y, z]), within a search distance. Returns (mp2p_icp.PlanePatch, absolute point-to-plane distance), or None.
        """
    def optionsByName(self) -> dict[str, mrpt.config.CLoadableOptions]:
        """
        A dict {name: options} with all the option groups of the map, e.g. 'insertionOptions'. They are the live objects, except that 'creationOptions' must not be modified in place: use trySetCreationOptions().
        """
    def queryField(self, point: typing.Any) -> typing.Any:
        """
        Trilinear interpolation of the field at a point. Returns (signed distance [m], unnormalized gradient as a float32 array), or None where the field is not defined.
        """
    def queryFieldBatch(self, points: typing.Any) -> dict:
        """
        queryField() for each row of an (N,3) array. Returns a dict with 'valid' (bool), 'dist' (float32) and 'gradient' ((N,3) float32), NaN where not valid. Releases the GIL.
        """
    def setVoxelProperties(self, voxel_size: float) -> None:
        """
        Changes the voxel size [m], CLEARING the map.
        """
    def size(self) -> int:
        """
        Number of voxels.
        """
    @typing.overload
    def trySetCreationOptions(self, options: TSDF.TCreationOptions) -> bool:
        """
        Applies new creation options. Returns False, leaving the map untouched, if they cannot be applied without discarding its contents (e.g. a different voxel size).
        """
    @typing.overload
    def trySetCreationOptions(self, cfg: mrpt.config.CConfigFileBase, section: str) -> bool:
        """
        Like trySetCreationOptions(options), reading them from a section of a configuration file (mrpt.config.CConfigFileBase).
        """
    def voxelSize(self) -> float:
        """
        Voxel size [m].
        """
    @property
    def creationOptions(self) -> TSDF.TCreationOptions:
        """
        Options that define the map structure (TCreationOptions). Reading it returns a COPY: modify it and assign it back, which applies it as trySetCreationOptions() does, raising ValueError if it is refused.
        """
    @creationOptions.setter
    def creationOptions(self, arg1: TSDF.TCreationOptions) -> None:
        ...
    @property
    def insertionOptions(self) -> TSDF.TInsertionOptions:
        """
        Insertion options (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: TSDF.TInsertionOptions) -> None:
        ...
    @property
    def renderOptions(self) -> TSDF.TRenderOptions:
        """
        Render options (TRenderOptions).
        """
    @renderOptions.setter
    def renderOptions(self, arg0: TSDF.TRenderOptions) -> None:
        ...
class KeyframePointCloudMap(mrpt.maps.CMetricMap):
    """
    A map of keyframes: point clouds in their own local frame, each with its global pose, so a loop closure only updates poses (mola::KeyframePointCloudMap). Each inserted point cloud observation becomes a new keyframe at the robot pose.
    """
    class TCreationOptions(mrpt.config.CLoadableOptions):
        """
        Options of the keyframe search and the per-point covariances.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def approximate_cov(self) -> bool:
            """
            Cov-to-cov matching against each active keyframe instead of a merged submap.
            """
        @approximate_cov.setter
        def approximate_cov(self, arg0: bool) -> None:
            ...
        @property
        def density_penalty_max_m(self) -> float:
            """
            Maximum proximity penalty [m], for an empty keyframe.
            """
        @density_penalty_max_m.setter
        def density_penalty_max_m(self, arg0: float) -> None:
            ...
        @property
        def density_penalty_min_points(self) -> int:
            """
            Keyframes with fewer points are penalized in the proximity ranking (0: disabled).
            """
        @density_penalty_min_points.setter
        def density_penalty_min_points(self, arg0: int) -> None:
            ...
        @property
        def k_correspondences_for_cov(self) -> int:
            """
            Number of neighbors to estimate each point covariance.
            """
        @k_correspondences_for_cov.setter
        def k_correspondences_for_cov(self, arg0: int) -> None:
            ...
        @property
        def max_distance_for_cov(self) -> float:
            """
            Maximum distance [m] of the neighbors used for a covariance.
            """
        @max_distance_for_cov.setter
        def max_distance_for_cov(self, arg0: float) -> None:
            ...
        @property
        def max_plane_deviation_for_cov(self) -> float:
            """
            Maximum distance [m] of a neighbor to the fitted plane for the plane regularization to apply (0: no test).
            """
        @max_plane_deviation_for_cov.setter
        def max_plane_deviation_for_cov(self, arg0: float) -> None:
            ...
        @property
        def max_search_keyframes(self) -> int:
            """
            Maximum number of keyframes to search for neighbors.
            """
        @max_search_keyframes.setter
        def max_search_keyframes(self, arg0: int) -> None:
            ...
        @property
        def max_view_angle_deg(self) -> float:
            """
            Maximum angle [deg] between view directions, for ViewDirectionFilter.MaxAngle.
            """
        @max_view_angle_deg.setter
        def max_view_angle_deg(self, arg0: float) -> None:
            ...
        @property
        def min_correspondences_for_cov(self) -> int:
            """
            Below this number of neighbors, an isotropic covariance is used.
            """
        @min_correspondences_for_cov.setter
        def min_correspondences_for_cov(self, arg0: int) -> None:
            ...
        @property
        def num_diverse_keyframes(self) -> int:
            """
            Keyframe search slots reserved for angularly diverse or farther keyframes.
            """
        @num_diverse_keyframes.setter
        def num_diverse_keyframes(self, arg0: int) -> None:
            ...
        @property
        def plane_regularization_lambda(self) -> float:
            """
            Variance along the normal of the regularized covariances (<=0: no regularization).
            """
        @plane_regularization_lambda.setter
        def plane_regularization_lambda(self, arg0: float) -> None:
            ...
        @property
        def rotation_distance_weight(self) -> float:
            """
            Weight [m/rad] of the angular distance when ranking keyframes by proximity.
            """
        @rotation_distance_weight.setter
        def rotation_distance_weight(self, arg0: float) -> None:
            ...
        @property
        def serialize_covariances(self) -> bool:
            """
            Store the per-point covariances in the serialized map.
            """
        @serialize_covariances.setter
        def serialize_covariances(self, arg0: bool) -> None:
            ...
        @property
        def serialize_kdtrees(self) -> bool:
            """
            Store the per-keyframe k-d trees in the serialized map.
            """
        @serialize_kdtrees.setter
        def serialize_kdtrees(self, arg0: bool) -> None:
            ...
        @property
        def use_view_direction_filter(self) -> bool:
            """
            Master switch of the view-direction filter of cov-to-cov pairs.
            """
        @use_view_direction_filter.setter
        def use_view_direction_filter(self, arg0: bool) -> None:
            ...
        @property
        def view_direction_filter(self) -> ViewDirectionFilter:
            """
            How the view-direction filter decides (ViewDirectionFilter).
            """
        @view_direction_filter.setter
        def view_direction_filter(self, arg0: ViewDirectionFilter) -> None:
            ...
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def remove_frames_farther_than(self) -> float:
            """
            If > 0, keyframes farther than this distance [m] from the robot are removed on each insertion.
            """
        @remove_frames_farther_than.setter
        def remove_frames_farther_than(self, arg0: float) -> None:
            ...
    class TLikelihoodOptions(mrpt.config.CLoadableOptions):
        """
        Options for computing the likelihood of observations (none yet).
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
    class TRenderOptions(mrpt.config.CLoadableOptions):
        """
        Options for rendering.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def color(self) -> mrpt.img.TColorf:
            """
            Color of points (mrpt.img.TColorf), if colormap is cmNONE.
            """
        @color.setter
        def color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def colormap(self) -> mrpt.img.TColormap:
            """
            Colormap (mrpt.img.TColormap).
            """
        @colormap.setter
        def colormap(self, arg0: mrpt.img.TColormap) -> None:
            ...
        @property
        def cov_color(self) -> mrpt.img.TColorf:
            """
            Color of the covariance ellipsoids (mrpt.img.TColorf).
            """
        @cov_color.setter
        def cov_color(self, arg0: mrpt.img.TColorf) -> None:
            ...
        @property
        def keyframes_axes_length(self) -> float:
            """
            If > 0, draw the axes of each keyframe, of this length [m].
            """
        @keyframes_axes_length.setter
        def keyframes_axes_length(self, arg0: float) -> None:
            ...
        @property
        def max_overall_points(self) -> int:
            """
            Maximum points to render overall.
            """
        @max_overall_points.setter
        def max_overall_points(self, arg0: int) -> None:
            ...
        @property
        def max_points_per_kf(self) -> int:
            """
            Maximum points to render per keyframe.
            """
        @max_points_per_kf.setter
        def max_points_per_kf(self, arg0: int) -> None:
            ...
        @property
        def point_size(self) -> float:
            """
            Point size [pixels].
            """
        @point_size.setter
        def point_size(self, arg0: float) -> None:
            ...
        @property
        def recolorByPointField(self) -> str:
            """
            Point field used for the colormap.
            """
        @recolorByPointField.setter
        def recolorByPointField(self, arg0: str) -> None:
            ...
        @property
        def show_cov_decimation(self) -> int:
            """
            Draw the covariance of one out of N points.
            """
        @show_cov_decimation.setter
        def show_cov_decimation(self, arg0: int) -> None:
            ...
        @property
        def show_covariances(self) -> bool:
            """
            Draw the per-point covariances as ellipsoids.
            """
        @show_covariances.setter
        def show_covariances(self, arg0: bool) -> None:
            ...
    class RegroupParams:
        """
        Parameters of regroupKeyframes(), see the C++ documentation.
        """
        def __init__(self) -> None:
            """
            Default parameters.
            """
        def __repr__(self) -> str:
            ...
        @property
        def core_fraction(self) -> float:
            """
            Inner-core fraction (0,1] of the extent of a group; the rest overlaps with others.
            """
        @core_fraction.setter
        def core_fraction(self, arg0: float) -> None:
            ...
        @property
        def edge_overlap(self) -> float:
            """
            Minimum overlap [0,1] of two keyframes to be merged into the same group.
            """
        @edge_overlap.setter
        def edge_overlap(self, arg0: float) -> None:
            ...
        @property
        def extent_factor(self) -> float:
            """
            Maximum extent of a group, as a multiple of the sensing radius of its seed.
            """
        @extent_factor.setter
        def extent_factor(self, arg0: float) -> None:
            ...
        @property
        def island_merge_fraction(self) -> float:
            """
            Groups smaller than this fraction of the largest one are merged into a neighbor.
            """
        @island_merge_fraction.setter
        def island_merge_fraction(self, arg0: float) -> None:
            ...
        @property
        def merge_decimate_voxel(self) -> float:
            """
            If > 0, voxel size [m] to decimate the merged clouds.
            """
        @merge_decimate_voxel.setter
        def merge_decimate_voxel(self, arg0: float) -> None:
            ...
        @property
        def unify_all(self) -> bool:
            """
            Merge all keyframes into a single one.
            """
        @unify_all.setter
        def unify_all(self, arg0: bool) -> None:
            ...
        @property
        def voxel_size(self) -> float:
            """
            Voxel size [m] to compute cloud overlaps (<=0: automatic).
            """
        @voxel_size.setter
        def voxel_size(self, arg0: float) -> None:
            ...
    def __copy__(self) -> KeyframePointCloudMap:
        """
        A deep copy of the map.
        """
    def __deepcopy__(self, memo: typing.Any) -> KeyframePointCloudMap:
        """
        A deep copy of the map.
        """
    def __getstate__(self) -> bytes:
        ...
    def __init__(self) -> None:
        """
        Creates an empty map.
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def drainEvictedKeyFrameIDs(self) -> list[int]:
        """
        Returns and clears the list of keyframe IDs removed since the last call.
        """
    def getKeyframePosesAsNumpy(self) -> tuple:
        """
        The keyframe IDs and poses, as a tuple (ids: uint64 array, poses: (N,4,4) float64 array of homogeneous matrices), sorted by ID.
        """
    def getPointsAsNumpy(self) -> numpy.ndarray:
        """
        The points of all keyframes, in the global frame, as an (N,3) float32 array. As the point cloud view of the map (e.g. mp2p_icp.layer_to_numpy()), they are decimated as set by renderOptions.max_points_per_kf and max_overall_points (0: no limit).
        """
    def icp_cleanup(self) -> None:
        """
        Releases the active submap of ICP.
        """
    def icp_get_prepared_as_global(self, pose: typing.Any, local_map_roi: mrpt.math.TBoundingBoxf | None = None) -> None:
        """
        Selects the keyframes near a pose (and an optional mrpt.math.TBoundingBoxf around it) as the active submap for ICP. Needed before the nn_*() queries.
        """
    def insertPointCloud(self, points: typing.Any, robotPose: typing.Any = None) -> bool:
        """
        Inserts a point cloud, as insertObservation() with an mrpt.obs.CObservationPointCloud. `points` (an (N,3) array, a dict {field name: 1-D array} as in mp2p_icp.layer_from_numpy(), or an mrpt.maps.CPointsMap, which is copied) are given in the robot frame, and the robot is at `robotPose` in the map: None (identity), mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 matrix or [x, y, z, yaw, pitch, roll]. Maps that trace rays (TSDF, NDT) take the robot position as their origin. Insertion options and map-specific behavior apply, e.g. removal of far contents. Returns True if the map was updated.
        """
    def keyframePoses(self) -> dict[int, mrpt.poses.CPose3D]:
        """
        A dict {keyframe ID: mrpt.poses.CPose3D} with the global poses of all keyframes.
        """
    def lastInsertedKeyFrameID(self) -> int | None:
        """
        ID of the last inserted keyframe, or None if none was inserted since the map was created or cleared.
        """
    def merge_with(self, other: KeyframePointCloudMap, relative_pose: typing.Any = None) -> None:
        """
        Adds the keyframes of another map, optionally transformed by a relative pose.
        """
    def nextFreeKeyFrameID(self) -> int:
        """
        The ID that the next inserted keyframe will get.
        """
    def nn_multiple_search(self, query: typing.Any, N: int) -> tuple:
        """
        The N closest points to a 3D query point, sorted by distance. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_radius_search(self, query: typing.Any, search_radius_sqr: float, maxPoints: int = 0) -> tuple:
        """
        The points within a distance of a 3D query point. The radius is given squared; maxPoints=0 means no limit. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_single_search(self, query: typing.Any) -> typing.Any:
        """
        Closest point to a 3D query point (mrpt.math.TPoint3D or [x, y, z]). Returns (index or ID, point, squared distance), or None if none was found.
        """
    def nn_single_search_batch(self, queries: typing.Any) -> dict:
        """
        Closest point to each row of an (N,3) array of query points. Returns a dict with 'found' (bool), 'indices' (uint64), 'points' ((N,3) float32, NaN if not found) and 'dist_sqr' (float32, inf if not found). Releases the GIL.
        """
    def optionsByName(self) -> dict[str, mrpt.config.CLoadableOptions]:
        """
        A dict {name: options} with all the option groups of the map, e.g. 'insertionOptions'. They are the live objects, except that 'creationOptions' must not be modified in place: use trySetCreationOptions().
        """
    def point_count(self) -> int:
        """
        Total number of points in all keyframes.
        """
    def regroupKeyframes(self, params: KeyframePointCloudMap.RegroupParams = ..., log: typing.Callable[[str], None] = None) -> KeyframePointCloudMap:
        """
        Builds a NEW map whose keyframes are overlapping groups of the keyframes of this one, so localization needs one active keyframe at a time (as the mm-kf-regroup tool). `log`, if given, is called with progress messages.
        """
    def setKeyframePose(self, id: int, pose: typing.Any) -> None:
        """
        Changes the global pose of a keyframe (mrpt.poses.CPose3D, a 4x4 matrix...). Does nothing if it does not exist.
        """
    def transform_map_left_multiply(self, pose: typing.Any) -> None:
        """
        Left-multiplies all keyframe poses by a pose.
        """
    @typing.overload
    def trySetCreationOptions(self, options: KeyframePointCloudMap.TCreationOptions) -> bool:
        """
        Applies new creation options. Returns False, leaving the map untouched, if they cannot be applied without discarding its contents (e.g. a different voxel size).
        """
    @typing.overload
    def trySetCreationOptions(self, cfg: mrpt.config.CConfigFileBase, section: str) -> bool:
        """
        Like trySetCreationOptions(options), reading them from a section of a configuration file (mrpt.config.CConfigFileBase).
        """
    @property
    def creationOptions(self) -> KeyframePointCloudMap.TCreationOptions:
        """
        Options that define the map structure (TCreationOptions). Reading it returns a COPY: modify it and assign it back, which applies it as trySetCreationOptions() does, raising ValueError if it is refused.
        """
    @creationOptions.setter
    def creationOptions(self, arg1: KeyframePointCloudMap.TCreationOptions) -> None:
        ...
    @property
    def insertionOptions(self) -> KeyframePointCloudMap.TInsertionOptions:
        """
        Insertion options (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: KeyframePointCloudMap.TInsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> KeyframePointCloudMap.TLikelihoodOptions:
        """
        Likelihood options (TLikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: KeyframePointCloudMap.TLikelihoodOptions) -> None:
        ...
    @property
    def renderOptions(self) -> KeyframePointCloudMap.TRenderOptions:
        """
        Render options (TRenderOptions).
        """
    @renderOptions.setter
    def renderOptions(self, arg0: KeyframePointCloudMap.TRenderOptions) -> None:
        ...
class IncrementalPointCloud(mrpt.maps.CGenericPointsMap):
    """
    A sliding-window point cloud for LiDAR odometry, with an incremental spatial index (mola::IncrementalPointCloud). It is an mrpt.maps.CGenericPointsMap, but point indices are storage slots: size() and the inherited NumPy conversions include freed slots (NaN) and removed points not reclaimed yet. Use livePointCount() and liveCompactedCopy() (or compact()) to work with the live points only.
    """
    class TCreationOptions(mrpt.config.CLoadableOptions):
        """
        Options of the spatial index, the sliding window and the per-point covariances.
        """
        def __init__(self) -> None:
            """
            Default options.
            """
        def __repr__(self) -> str:
            """
            Options as Name(field=value, ...).
            """
        def __str__(self) -> str:
            """
            Options as human-readable text.
            """
        @property
        def alpha_balance(self) -> float:
            """
            KDTree index only: balance rebuild threshold.
            """
        @alpha_balance.setter
        def alpha_balance(self, arg0: float) -> None:
            ...
        @property
        def alpha_deleted(self) -> float:
            """
            KDTree index only: deleted points threshold.
            """
        @alpha_deleted.setter
        def alpha_deleted(self, arg0: float) -> None:
            ...
        @property
        def async_rebuild(self) -> bool:
            """
            KDTree index only: rebuild the tree in a background thread.
            """
        @async_rebuild.setter
        def async_rebuild(self, arg0: bool) -> None:
            ...
        @property
        def index_type(self) -> PointIndexType:
            """
            Spatial index type (PointIndexType).
            """
        @index_type.setter
        def index_type(self, arg0: PointIndexType) -> None:
            ...
        @property
        def index_voxel_size(self) -> float:
            """
            Voxel size [m] of the VoxelHash index.
            """
        @index_voxel_size.setter
        def index_voxel_size(self, arg0: float) -> None:
            ...
        @property
        def k_correspondences_for_cov(self) -> int:
            """
            Number of neighbors to estimate each point covariance.
            """
        @k_correspondences_for_cov.setter
        def k_correspondences_for_cov(self, arg0: int) -> None:
            ...
        @property
        def max_distance_for_cov(self) -> float:
            """
            Maximum distance [m] of the neighbors used for a covariance.
            """
        @max_distance_for_cov.setter
        def max_distance_for_cov(self, arg0: float) -> None:
            ...
        @property
        def max_plane_deviation_for_cov(self) -> float:
            """
            Maximum distance [m] of a neighbor to the fitted plane for the plane regularization to apply (0: no test).
            """
        @max_plane_deviation_for_cov.setter
        def max_plane_deviation_for_cov(self, arg0: float) -> None:
            ...
        @property
        def max_view_angle_deg(self) -> float:
            """
            Maximum angle [deg] between view directions, for ViewDirectionFilter.MaxAngle.
            """
        @max_view_angle_deg.setter
        def max_view_angle_deg(self, arg0: float) -> None:
            ...
        @property
        def min_correspondences_for_cov(self) -> int:
            """
            Below this number of neighbors, an isotropic covariance is used.
            """
        @min_correspondences_for_cov.setter
        def min_correspondences_for_cov(self, arg0: int) -> None:
            ...
        @property
        def min_distance_between_points(self) -> float:
            """
            If not 0, inserted points closer than this [m] to one in the map are dropped.
            """
        @min_distance_between_points.setter
        def min_distance_between_points(self, arg0: float) -> None:
            ...
        @property
        def min_neighbors_to_cache_cov(self) -> int:
            """
            Minimum neighbors for a covariance to be cached (0: k_correspondences_for_cov).
            """
        @min_neighbors_to_cache_cov.setter
        def min_neighbors_to_cache_cov(self, arg0: int) -> None:
            ...
        @property
        def plane_regularization_lambda(self) -> float:
            """
            Variance along the normal of the regularized covariances (<=0: no regularization).
            """
        @plane_regularization_lambda.setter
        def plane_regularization_lambda(self, arg0: float) -> None:
            ...
        @property
        def remove_points_farther_than(self) -> float:
            """
            If not 0, on each insertion keep only the points in a cube of this half side [m] around the robot.
            """
        @remove_points_farther_than.setter
        def remove_points_farther_than(self, arg0: float) -> None:
            ...
        @property
        def reserve_points(self) -> int:
            """
            If not 0, preallocate storage for this many points.
            """
        @reserve_points.setter
        def reserve_points(self, arg0: int) -> None:
            ...
        @property
        def serialize_kdtree(self) -> bool:
            """
            KDTree index only: store the tree in the serialized map.
            """
        @serialize_kdtree.setter
        def serialize_kdtree(self, arg0: bool) -> None:
            ...
        @property
        def use_view_direction_filter(self) -> bool:
            """
            Master switch of the view-direction filter of cov-to-cov pairs.
            """
        @use_view_direction_filter.setter
        def use_view_direction_filter(self, arg0: bool) -> None:
            ...
        @property
        def view_direction_filter(self) -> ViewDirectionFilter:
            """
            How the view-direction filter decides (ViewDirectionFilter).
            """
        @view_direction_filter.setter
        def view_direction_filter(self, arg0: ViewDirectionFilter) -> None:
            ...
    def __copy__(self) -> IncrementalPointCloud:
        """
        A deep copy of the map.
        """
    def __deepcopy__(self, memo: typing.Any) -> IncrementalPointCloud:
        """
        A deep copy of the map.
        """
    def __getstate__(self) -> bytes:
        ...
    def __init__(self) -> None:
        """
        Creates an empty map.
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def cachedCovarianceCount(self) -> int:
        """
        Number of per-point covariances in the cache.
        """
    @typing.overload
    def changeCoordinatesReference(self, pose: mrpt.poses.CPose3D) -> None:
        """
        Transforms all points by a pose (mrpt.poses.CPose3D) and rebuilds the index.
        """
    @typing.overload
    def changeCoordinatesReference(self, pose: mrpt.poses.CPose2D) -> None:
        """
        Transforms all points by a pose (mrpt.poses.CPose2D) and rebuilds the index.
        """
    def compact(self) -> None:
        """
        Drops the freed storage slots, so size() equals livePointCount(), and rebuilds the index. Invalidates the indices returned by the nn_*() methods.
        """
    def covarianceComputations(self) -> int:
        """
        Number of per-point covariance estimations so far (diagnostics).
        """
    def insertPointCloud(self, points: typing.Any, robotPose: typing.Any = None) -> bool:
        """
        Inserts a point cloud, as insertObservation() with an mrpt.obs.CObservationPointCloud. `points` (an (N,3) array, a dict {field name: 1-D array} as in mp2p_icp.layer_from_numpy(), or an mrpt.maps.CPointsMap, which is copied) are given in the robot frame, and the robot is at `robotPose` in the map: None (identity), mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 matrix or [x, y, z, yaw, pitch, roll]. Maps that trace rays (TSDF, NDT) take the robot position as their origin. Insertion options and map-specific behavior apply, e.g. removal of far contents. Returns True if the map was updated.
        """
    def keepOnlyPointsNear(self, center: typing.Any, half_side: float) -> None:
        """
        Removes the points outside a cube of half side `half_side` [m] around `center`.
        """
    def liveCompactedCopy(self) -> mrpt.maps.CGenericPointsMap:
        """
        A new mrpt.maps.CGenericPointsMap with the live points only, with all their fields.
        """
    def livePointCount(self) -> int:
        """
        Number of points in the map, without the freed storage slots counted by size().
        """
    def nn_multiple_search(self, query: typing.Any, N: int) -> tuple:
        """
        The N closest points to a 3D query point, sorted by distance. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_radius_search(self, query: typing.Any, search_radius_sqr: float, maxPoints: int = 0) -> tuple:
        """
        The points within a distance of a 3D query point. The radius is given squared; maxPoints=0 means no limit. Returns (indices or IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).
        """
    def nn_single_search(self, query: typing.Any) -> typing.Any:
        """
        Closest point to a 3D query point (mrpt.math.TPoint3D or [x, y, z]). Returns (index or ID, point, squared distance), or None if none was found.
        """
    def nn_single_search_batch(self, queries: typing.Any) -> dict:
        """
        Closest point to each row of an (N,3) array of query points. Returns a dict with 'found' (bool), 'indices' (uint64), 'points' ((N,3) float32, NaN if not found) and 'dist_sqr' (float32, inf if not found). Releases the GIL.
        """
    def optionsByName(self) -> dict[str, mrpt.config.CLoadableOptions]:
        """
        A dict {name: options} with all the option groups of the map, e.g. 'insertionOptions'. They are the live objects, except that 'creationOptions' must not be modified in place: use trySetCreationOptions().
        """
    def recyclableSlotCount(self) -> int:
        """
        Number of storage slots ready to be reused by the next insertion.
        """
    @typing.overload
    def trySetCreationOptions(self, options: IncrementalPointCloud.TCreationOptions) -> bool:
        """
        Applies new creation options. Returns False, leaving the map untouched, if they cannot be applied without discarding its contents (e.g. a different voxel size).
        """
    @typing.overload
    def trySetCreationOptions(self, cfg: mrpt.config.CConfigFileBase, section: str) -> bool:
        """
        Like trySetCreationOptions(options), reading them from a section of a configuration file (mrpt.config.CConfigFileBase).
        """
    @property
    def creationOptions(self) -> IncrementalPointCloud.TCreationOptions:
        """
        Options that define the map structure (TCreationOptions). Reading it returns a COPY: modify it and assign it back, which applies it as trySetCreationOptions() does, raising ValueError if it is refused.
        """
    @creationOptions.setter
    def creationOptions(self, arg1: IncrementalPointCloud.TCreationOptions) -> None:
        ...
class OccGrid(mrpt.serialization.CSerializable):
    """
    A 2D occupancy grid (mrpt.maps.COccupancyGridMap2D) with a super-resolution likelihood cache (mola::OccGrid). Experimental: the cache is not used yet.
    """
    class InsertionParameters:
        """
        Parameters for inserting range scans.
        """
        def __init__(self) -> None:
            """
            Default parameters.
            """
        def __repr__(self) -> str:
            ...
        @property
        def considerInvalidRangesAsFreeSpace(self) -> bool:
            """
            Use invalid ranges as free space, if their neighbor rays are also invalid.
            """
        @considerInvalidRangesAsFreeSpace.setter
        def considerInvalidRangesAsFreeSpace(self, arg0: bool) -> None:
            ...
        @property
        def decimation(self) -> int:
            """
            Use one out of N rays.
            """
        @decimation.setter
        def decimation(self, arg0: int) -> None:
            ...
        @property
        def maxDistanceInsertion(self) -> float:
            """
            Largest distance [m] at which cells are updated.
            """
        @maxDistanceInsertion.setter
        def maxDistanceInsertion(self, arg0: float) -> None:
            ...
        @property
        def maxFreenessInvalidRanges(self) -> float:
            """
            Like maxFreenessUpdateCertainty, for invalid ranges (0: maxOccupancyUpdateCertainty).
            """
        @maxFreenessInvalidRanges.setter
        def maxFreenessInvalidRanges(self, arg0: float) -> None:
            ...
        @property
        def maxFreenessUpdateCertainty(self) -> float:
            """
            Value in [0.5, 1] for the update of free cells (0: maxOccupancyUpdateCertainty).
            """
        @maxFreenessUpdateCertainty.setter
        def maxFreenessUpdateCertainty(self, arg0: float) -> None:
            ...
        @property
        def maxOccupancyUpdateCertainty(self) -> float:
            """
            Value in [0.5, 1] for the Bayesian update of occupied cells.
            """
        @maxOccupancyUpdateCertainty.setter
        def maxOccupancyUpdateCertainty(self, arg0: float) -> None:
            ...
        @property
        def wideningBeamsWithDistance(self) -> bool:
            """
            Rays widen with distance, like real laser beams.
            """
        @wideningBeamsWithDistance.setter
        def wideningBeamsWithDistance(self, arg0: bool) -> None:
            ...
    class LikelihoodParameters:
        """
        Parameters of the likelihood cache.
        """
        def __init__(self) -> None:
            """
            Default parameters.
            """
        def __repr__(self) -> str:
            ...
        @property
        def superResolutionFactor(self) -> float:
            """
            Resolution of the likelihood cache, as a factor of that of the grid.
            """
        @superResolutionFactor.setter
        def superResolutionFactor(self, arg0: float) -> None:
            ...
    def __copy__(self) -> OccGrid:
        """
        A deep copy of the map.
        """
    def __deepcopy__(self, memo: typing.Any) -> OccGrid:
        """
        A deep copy of the map.
        """
    def __getstate__(self) -> bytes:
        ...
    def __init__(self) -> None:
        """
        Creates an empty grid.
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def grid(self) -> mrpt.maps.COccupancyGridMap2D:
        """
        The occupancy grid (mrpt.maps.COccupancyGridMap2D).
        """
    def insertObservation(self, obs: mrpt.obs.CObservation2DRangeScan, robotPose: mrpt.math.TPose2D) -> None:
        """
        Inserts a 2D range scan (mrpt.obs.CObservation2DRangeScan) taken from a robot pose (mrpt.math.TPose2D).
        """
    def resetLikelihoodCache(self) -> None:
        """
        Invalidates all cached likelihood values.
        """
    def resizeGrid(self, min_corner: typing.Any, max_corner: typing.Any, new_cells_occupancy: float = 0.5) -> None:
        """
        Changes the grid size, keeping its contents.
        """
    def setSize(self, min_corner: typing.Any, max_corner: typing.Any, resolution: float, occupancy: float = 0.5) -> None:
        """
        Changes the grid size, ERASING its contents. Corners are mrpt.math.TPoint2D or [x, y] [m], the resolution is the cell size [m].
        """
    @property
    def insertionParameters(self) -> OccGrid.InsertionParameters:
        """
        Parameters for inserting range scans (InsertionParameters).
        """
    @insertionParameters.setter
    def insertionParameters(self, arg0: OccGrid.InsertionParameters) -> None:
        ...
    @property
    def likelihoodParameters(self) -> OccGrid.LikelihoodParameters:
        """
        Parameters of the likelihood cache (LikelihoodParameters).
        """
    @likelihoodParameters.setter
    def likelihoodParameters(self, arg0: OccGrid.LikelihoodParameters) -> None:
        ...
HAS_INCREMENTAL_KDTREE: bool = True
HAS_INCREMENTAL_KDTREE_BAKE: bool = True
__version__: str = '3.4.0'
