Create a custom backend
=======================

0. Preparation
--------------
Before you proceed, check the :ref:`Installing the Pegasus Simulator` section first, if you haven't already.


1. Code
-------

The tutorial corresponds to the ``4_python_single_vehicle.py`` example in the ``examples`` directory.

:cite:p:`Pinto2021`, :cite:p:`mellinger_kumar`


.. literalinclude:: ../../../examples/4_python_single_vehicle.py
   :language: python
   :emphasize-lines: 30-31,68-74
   :linenos:

The tutorial corresponds to the ``nonlinear_controller.py`` example in the ``examples/utils`` directory.


.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :emphasize-lines: 6,13-23,78-81,92-95,110-118,121-127,135-141,144-150
   :linenos:

2. Explanation
--------------

.. literalinclude:: ../../../examples/4_python_single_vehicle.py
   :language: python
   :lines: 30-31
   :linenos:
   :lineno-start: 30

.. literalinclude:: ../../../examples/4_python_single_vehicle.py
   :language: python
   :lines: 68-74
   :linenos:
   :lineno-start: 68

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 6
   :linenos:
   :lineno-start: 6

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 13-23
   :linenos:
   :lineno-start: 13

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 78-81
   :linenos:
   :lineno-start: 78

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 92-95
   :linenos:
   :lineno-start: 92

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 110-118
   :linenos:
   :lineno-start: 110

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 121-127
   :linenos:
   :lineno-start: 121

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 135-141
   :linenos:
   :lineno-start: 135

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 144-150
   :linenos:
   :lineno-start: 144

3. Execution
------------

Now let's run the Python script:

.. code:: bash

   ISAACSIM_PYTHON examples/4_python_single_vehicle.py