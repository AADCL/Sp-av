Fast-Planner source: https://github.com/HKUST-Aerial-Robotics/Fast-Planner
Commit: 41be219fe4ecc43bf0e0c2b42a523f8755ccc0bd
Vendored core: KinodynamicAstar, NonUniformBspline, BsplineOptimizer.
Upstream root GPL-3.0 license and per-file LGPL-3.0-or-later notices retained.
Integration replaces plan_env with the project's bounded observed-voxel distance field.
No upstream flight FSM, simulation node or trajectory server is started.
Patches: initialized static search clocks, corrected map upper boundary,
refreshed mutable priority queue scores, bounded search wall time,
rejected invalid shot duration, retained optimizer seed on early failure.
The integration uniformly scales spline knots using derivative control bounds
and independently validates the continuous swept path and position reference.
