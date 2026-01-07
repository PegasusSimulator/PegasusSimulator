C3BF-Based Safe Quadrotor Control
=================================


Overview
--------
This example demonstrates the integration of a Collision Cone Control Barrier
Function (C3BF) with a nonlinear quadrotor controller in Pegasus Simulator.
The C3BF acts as a velocity-level safety filter that enforces obstacle avoidance
while preserving aggressive trajectory tracking.

Directory Structure
-------------------
Relevant files introduced in this contribution:

- ``examples/utils/c3bf.py``  
  Implements the collision cone CBF formulation.

- ``examples/utils/nonlinear_controller_with_c3bf.py``  
  Nonlinear controller augmented with the C3BF safety filter.

- ``examples/report_for_cbf/``  
  Detailed mathematical derivations, experiments, and discussion.

- ``examples/result_videos_cbf/``  
  Baseline vs C3BF comparison videos.

How to Run the Example
----------------------

To enable the C3BF controller:

1. Open the example script:
   ``examples/4_python_single_vehicle.py``

2. Modify the controller import:

   .. code-block:: python

      # Original
      from nonlinear_controller import NonlinearController

      # Replace with
      from nonlinear_controller_with_c3bf import NonlinearController

3. Launch the simulation using Isaac Sim:

   .. code-block:: bash

      isaac_run examples/4_python_single_vehicle.py

The quadrotor will now execute the same minimum-snap trajectory while enforcing
real-time collision avoidance.

Results
-------
The C3BF-enabled controller successfully avoids collisions in all tested
scenarios, including close-proximity spherical obstacles, while the baseline
controller experiences head-on collisions under identical conditions.

Refer to:
- ``examples/report_for_cbf`` for quantitative analysis
- ``examples/result_videos_cbf`` for visual comparisons
