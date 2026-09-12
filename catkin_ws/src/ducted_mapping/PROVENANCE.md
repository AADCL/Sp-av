Source: https://github.com/AADCL/AG-TEST
Commit: 9a309a93fd900abccb8775ebdb727c65946f5c0d
License: Apache-2.0 (LICENSE.AG-TEST).
Ported StaticMapFilter and Point3d/VoxelAccumulator headers and upstream filter test.
Changes: namespace, finite/bounded validation, monotonic scan stamps.
Added loop-corrected scan archive, explicit observed-space voxel ray traversal,
resource-limited replay, metadata and no-overwrite atomic map bundle publication.
Real-time localization/obstacle scans are not filtered by this static-map export.
Export preparation uses bounded parallel reads/radius searches with ordered
temporal updates. Native static output is 5 cm; Bayesian membership thresholds
remain unchanged. The separate observed-space grid is opt-in because EGO uses
live scans. Metadata v2 explicitly records whether that grid was exported.
