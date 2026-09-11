# EGO-Planner source and integration

Official source: https://github.com/ZJU-FAST-Lab/ego-planner
Pinned commit: `bfda51284c8c1b476043255a8145ef925a3778a5` (ROS 1).
License: GPL-3.0; full upstream LICENSE is adjacent. `lbfgs.hpp` retains its embedded license.

Copied core: `bspline_opt/{bspline_optimizer,uniform_bspline,lbfgs}` and `path_searching/dyn_a_star`. The original global waypoint trajectory manager, swarm logic, simulator, flight FSM and command sender are not used.

The wrapper constructs a bounded rolling occupancy snapshot from live complete obstacle points. It calls the actual EGO `initControlPoints` and `BsplineOptimizeTrajRebound`; its short A* searches supply local collision rebound directions. There is no independent global planner, ESDF or saved-map input.

Integration changes to upstream:
- Replace GridMap's ROS/depth environment with the occupancy-only snapshot adapter in `include/plan_env/grid_map.h`.
- Check the complete local seed and spline, extending the upstream near-two-thirds checks. A short trajectory around the wide ducted body otherwise never sees the obstacle exit and cannot construct rebound gradients.
- Guard three A* index advances before dereference; only use an intersection computed for the current control point; do not propagate empty rebound vectors.
- Initialize infinity correctly, initialize the search endpoint before its heuristic, clear stale paths, restore priority ordering after key decrease, and release all search-pool allocations.
- Use a monotonic snapshot deadline in local search and the optimizer cancellation callback.
- Omit the unused gradient-descent header whose integer defaults overflow; the actual solver is L-BFGS.

Wrapper differences:
- Quintic boundary seed, exact initial position/velocity and stopped local endpoint.
- Conservative body, braking and voxel inflation; measured AGL limits in absolute odom Z.
- Uniform knot scaling bounds continuous spline speed/acceleration by derivative control points. The validated short position chord is passed to the existing position controller, not velocity/acceleration feed-forward.
- Full-curve collision checks and final freshness validation; no command is sent on failure.
- The rolling cloud mode treats cells without returns inside its configured local sensing horizon as unoccupied, as in point-cloud EGO use. It does not certify unknown or occluded free space. Outside the local horizon is blocked; navigation is limited by live sensing and cannot solve large/global detours.
