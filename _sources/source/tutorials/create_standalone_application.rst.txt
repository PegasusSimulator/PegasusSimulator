Create a Standalone Application
===============================

This tutorial introduces how to create a standalone python script to set up an empty scene.
It introduces the main class used by ``Isaac Sim`` simulator, :class:`SimulationApp`, as well as the :class:`timeline`
concept that helps to launch and control the simulation timeline. In this tutorial we do **NOT** expose the 
``Pegasus API`` yet.

1. Code
-------

The tutorial corresponds to the ``0_template_app.py`` example in the ``examples`` directory.


.. literalinclude:: ../../../examples/0_template_app.py
   :language: python
   :emphasize-lines: 11-16,35-54,85-102
   :linenos:

2. Explanation
--------------

The first step, when creating a ``standalone application`` with ``Isaac Sim`` is to instantiate the :class:`SimulationApp`
object, and it takes a dictionary of parameters that can be used to configure the application. This 
object will be responsible for opening a ``bare bones`` version of the simulator. The ``headless`` parameter
selects whether to launch the GUI window or not.


.. literalinclude:: ../../../examples/0_template_app.py
   :language: python
   :lines: 11-16
   :linenos:
   :lineno-start: 11

.. note::
   
   You can **NOT** import other omniverse modules before instantiating the :class:`SimulationApp`, otherwise the simulation crashes
   before the window it's even open. For a deeper understanding on how the :class:`SimulationApp` works, check 
   `NVIDIA's offical documentation <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.kit/docs/index.html#simulation-application-omni-isaac-kit>`__.

In order to create a `Simulation Context <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#module-omni.isaac.core.simulation_context>`__,
we resort to the `World <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.world.World>`__ class environment. The `World <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.world.World>`__  class inherits the `Simulation Context <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#module-omni.isaac.core.simulation_context>`__,
and provides already some default parameters for setting up a simulation for us. You can pass as arguments the physics 
time step and rendering time step (in seconds). The rendering and physics can be set to run at different rates. For now let's use the defaults: 60Hz for both rendering and physics.

After creating the `World <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.world.World>`__ class environment. The `World <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.world.World>`__  class inherits the `Simulation Context <https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#module-omni.isaac.core.simulation_context>`__, we will
take advantage of its ``callback system`` to declare that some functions defined by us should be called at every physics iteration, 
render iteration or every time there is a timeline event, such as pressing the start/stop button. In this case, the
``physics_callback`` method will be invoked at every physics step, the ``render_callback`` at every render step and 
``timeline_callback`` every time there is a timeline event. You can add as many callbacks as you want. After having all your
callbacks defined, the ``world.reset()`` method should be invoked to initialize the ``physics context`` and set any existing
robot joints (in this case there is none) to their default state.

.. literalinclude:: ../../../examples/0_template_app.py
   :language: python
   :lines: 35-54
   :linenos:
   :lineno-start: 35

To start the actual simulation, we invoke the timeline's ``play()`` method. This is necessary in order to ensure that 
every previously defined callback gets invoked. In order for the ``Isaac Sim`` app to remain responsive, we need to create
a ``while loop`` that invokes ``world.step(render=True)`` to make sure that the UI get's rendered.

.. literalinclude:: ../../../examples/0_template_app.py
   :language: python
   :lines: 85-102
   :linenos:
   :lineno-start: 85

As you may have noticed, our ``infinite loop`` is very clean. In this work, similarly to the ROS 2 standard,
we prefer to perform all the logic by resorting to function callbacks instead of cramming all the logic inside 
the while loop. This structure allows our code to be more organized and more modular. As you will learn in the following
tutorials, the ``Pegasus Simulator API`` is built on top of this idea of ``callbacks``.

3. Execution
------------

Now let's run the Python script:

.. code:: bash

   ISAACSIM_PYTHON examples/0_template_app.py

This should open a stage with a blue ground-plane. The simulation should start playing automatically and the stage being rendered. 
To stop the simulation, you can either ``close the window``, press the ``STOP button`` in the UI, or press ``Ctrl+C`` in the terminal.

.. note:: 

   Notice that the window will only close when pressing the ``STOP button``, because we defined a method called 
   ``timeline_callback`` that get's invoked for every ``timeline event``, such as pressing the ``STOP button``. 