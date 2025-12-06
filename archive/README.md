# Archive – Early Autonomy Nodes

This folder contains early development versions of the autonomy logic used
during the AeroHades 2025 project.

The final mission goal was always **payload delivery over a visual target**.
In the earliest versions, a simplified "landing on the target" behavior was
used as a proxy to test:

- Pixel-error → velocity mapping
- Mode switching (e.g. LOITER → OFFBOARD)
- Alignment and stabilization over a visual target
- Descent logic and recovery behavior

Once those control pathways were validated, the logic was refactored and the
behavior changed to what the final `hades_lander` package implements:
**descend to a configured altitude over the target and perform a payload drop**
instead of landing.

## v0_monolithic/
The earliest working attempt. A single script handling arming, mode switching,
target handling, descent, and "test landing" using flag-based logic.

## v1_transitional/
Improved version of v0 with somewhat cleaner internal logic and better arming
and mode-switch handling. Still monolithic and using landing for testing.

## v2_semi_fsm/
Introduces a semi-structured internal "state machine" using flags and conditions,
making the behavior easier to reason about. Landing is still used as a test
behavior.

## v3_pre_fsm/
The most refined monolithic version, with clearer phases and mission interaction,
used as a stepping stone toward the final ROS 2 node architecture and the
payload-drop behavior in `hades_lander`.

These files are preserved for historical and educational purposes only.
They are not part of the final system and are not intended for direct use.
