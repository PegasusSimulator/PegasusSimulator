Your First Simulation
=====================

In this tutorial, you will create a standalone Python application to perform a simulation using 1 vehicle and ``PX4-Autopilot``. 
The end result should be the equivalent to the :ref:`Run in Extension Mode (GUI)` tutorial. To achieve this, we will cover the
basics of the Pegasus Simulator :ref:`API Reference`.

0. Preparation
--------------
Before you proceed, make sure you have read the :ref:`Installing the Pegasus Simulator` section first and that your system
is correctly configured. Also, we will assume that you already followed the :ref:`Create a Standalone Application` tutorial, as in this tutorial
we will resort to concepts introduced in it.

Additionally, you will also be required to have ``QGroundControl`` installed to operate the simulated vehicle. If you haven't
already, follow the Preparation section in the :ref:`Run in Extension Mode (GUI)` tutorial.


1. Code
--------

The tutorial corresponds to the ``1_px4_single_vehicle.py`` example in the ``examples`` directory.


.. literalinclude:: ../../../examples/1_px4_single_vehicle.py
   :language: python
   :emphasize-lines: 24-29,47-48,55-56,58-77,79-80
   :linenos:

2. Explanation
---------------

In order to start using the functionalities provided by the ``Pegasus Simulator``, we import some of the most commonly
used classes from the ``pegasus.simulator.logic`` module, which contains all the core APIs. To check all the classes and 
methods currently provided, please refer to the :ref:`API Reference` section.

.. literalinclude:: ../../../examples/1_px4_single_vehicle.py
   :language: python
   :lines: 24-29
   :linenos:
   :lineno-start: 24

.. note::

   Some APIs may not be documented in the :ref:`API Reference` section. This means one of three things:
   i) they are not meant for public use; ii) under development (untested); iii) or just deprecated and about to be replaced.

Next, we initialize the :class:`PegasusInterface` object. This is a singleton, which means that there exists only one 
:class:`PegasusInterface` in memory at any time. This interface creates a `World <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.world.World>`__ class environment. The `World <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.world.World>`__  class inherits the `Simulation Context <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#module-omni.isaac.core.simulation_context>`__
internally, so you do not need to declare your own manually when using this API. By default, when physics engine is set to run
at ``250 Hz`` and the render engine at ``60Hz``.

The :class:`PegasusInterface` provides methods to set the simulation environment, define the geographic coordinates of the origin of the world, setting the default path for the PX4-Autopilot installation and much more. To learn more about this class, refer to the :ref:`Pegasus Interface` API section.

.. literalinclude:: ../../../examples/1_px4_single_vehicle.py
   :language: python
   :lines: 47-48
   :linenos:
   :lineno-start: 47

Next, we load the 3D world environment. For this purpose, we provide the auxiliary method ``load_environment(usd_file)`` for
loading pre-made environments stored in `Universal Scene Description (.usd) <https://developer.nvidia.com/usd>`__ files. In this
example, we load one of the simulation environments already provided by NVIDIA, whose path in stored in the ``SIMULATION_ENVIRONMENTS``
dictionary. To known all provided environments, check the :ref:`Params` API section.

.. literalinclude:: ../../../examples/1_px4_single_vehicle.py
   :language: python
   :lines: 55-56
   :linenos:
   :lineno-start: 55

The next step is to create a multirotor vehicle object. For that we start by creating a ``MultirotorConfig``, which contains 
by default a :class:`QuadraticThrustCurve`, :class:`Linear Drag` model, a list of ``Sensors`` composed of a :class:`GPS`, 
:class:`IMU`, :class:`Magnetometer` and :class:`Barometer`, and a list of control :class:`Backend`.

In this tutorial, we whish to control the vehicle via ``MAVLink``, ``PX4-Autopilot`` and ``QGroundControl``. Therefore,
we create a ``MAVLink`` control backend configured to also launch ``PX4-Autopilot`` in SITL mode in the background.

.. literalinclude:: ../../../examples/1_px4_single_vehicle.py
   :language: python
   :lines: 58-68
   :linenos:
   :lineno-start: 58

To create the actual ``Multirotor`` object, we must specify the ``stage_path``, i.e. the name that the vehicle will have inside
the simulation tree and the ``usd_path`` which defines where the 3D model of the vehicle is stored. Once again, to known all 
provided vehicle models, check the :ref:`Params` API section. Additionally, we can provide the original position and orientation
of the vehicle, according to an East-North-DOWN (ENU) convention.

.. literalinclude:: ../../../examples/1_px4_single_vehicle.py
   :language: python
   :lines: 70-77
   :linenos:
   :lineno-start: 70

After the simulation is setup, the ``world.reset()`` method should be invoked to initialize the ``physics context`` and set any existing
robot joints (the multirotor rotor joints) to their default state.

.. literalinclude:: ../../../examples/1_px4_single_vehicle.py
   :language: python
   :lines: 79-80
   :linenos:
   :lineno-start: 79


3. Execution
------------

Now let's run the Python script:

.. code:: bash

   ISAACSIM_PYTHON examples/1_px4_single_vehicle.py

This should open a stage with a blue ground-plane with an 3DR Iris vehicle model in it. The simulation should start playing automatically and the stage being rendered. 
PX4-Autopilot will start running automatically in the background, receiving data from the simulated sensors and sending
target angular velocities for the vehicle rotors. You can now play with the vehicle using ``QGroundControl`` similarly to 
what was shown in :ref:`Run in Extension Mode (GUI)` tutorial.

To stop the simulation, you can either ``close the window``, press the ``STOP button`` in the UI, or press ``Ctrl+C`` in the terminal.
