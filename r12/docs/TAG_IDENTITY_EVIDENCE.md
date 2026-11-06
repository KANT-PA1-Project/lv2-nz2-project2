# Tag identity evidence

`detect_tags(image, detector)` retains its legacy list result. Call `detect_tag_evidence(image, detector)` when a consumer must distinguish a clean single marker from hidden conflicts or weak groups.

The new result contains `tags`, per-location `groups`, and `ambiguous`. Group reasons include `agreed`, `conflicting_scale_ids`, `insufficient_scale_agreement`, and `unregistered_id`. A surviving agreed tag does not cancel a different ambiguous group in the same image. Unknown decoded IDs retain no mission scale. Destination IDs retain their destination role.

`IdentityReobservation(run_id)` tracks diagnostic visible-ID confirmation. Call `confirm(object_id, physics_step=current_step, run_id=current_run)` only after the caller has authenticated a fresh, settled capture. A repeated/reset epoch or run mismatch cannot provide another vote. `reject()` clears the streak without rewinding the latest epoch. The minimum is two new epochs with the same object ID; an ID change starts a new streak. Destination IDs are rejected as object targets.

This tracker does not authenticate sensors, validate freshness or geometry, distinguish all physical objects sharing an ID, or authorize movement. It always returns `motion_authorized=false`. Those checks remain the caller's responsibility. Existing consumers are not switched to a new wire schema or granted authority by importing this module.
