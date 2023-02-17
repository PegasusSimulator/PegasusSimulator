Create a Standalone Application
===============================

This tutorial introduces how to create a standalone python script to set up a simple empty scene.
It introduces the main class used by ``Isaac Sim`` simulator, :class:`SimulationApp`, as well as the :class:`timeline`
concept that helps to launch and control the simulation timeline respectively. In this tutorial we do **NOT** expose the 
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

.. literalinclude:: ../../../examples/0_template_app.py
   :language: python
   :lines: 11-16
   :linenos:
   :lineno-start: 11

.. literalinclude:: ../../../examples/0_template_app.py
   :language: python
   :lines: 35-54
   :linenos:
   :lineno-start: 35

.. literalinclude:: ../../../examples/0_template_app.py
   :language: python
   :lines: 85-102
   :linenos:
   :lineno-start: 85

3. Execution
------------

Now let's run the Python script:

.. code:: bash

   ISAACSIM_PYTHON examples/0_template_app.py

This should open a stage with a blue ground plane. The simulation should start playing automatically and the stage rendered. 
To stop the simulation, you can either ``close the window``, press the ``STOP button`` in the UI, or press ``Ctrl+C`` in the terminal.

.. note:: 

   Notice that the window will only close when pressing the STOP button, because we defined a method called 
   timeline_callback that get's invoked for every timeline event, such as pressing the STOP button. 