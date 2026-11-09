# Additional multi-vehicle development — QA deferred

- Use a sampled smooth fifth-order lateral shift for passing-side clearance
  rather than treating every centreline obstacle as a permanent block.
- Require measured current road boundaries throughout the shift and passing
  horizon. Reject unobserved space or vehicle-width intrusions.
- Check centre-lane obstacles again before the return merge.
- New pure traffic_guard.py offers closing-speed headway and pass-abort
  helpers. It does NOT constitute real object-ID tracking, prediction of all
  cars or certified multi-vehicle collision avoidance.
- Existing startup gate, approval flags, map usage and competition assets
  remain unchanged. Do NOT run unit tests or Gazebo until user requests QA.
