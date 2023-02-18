Development
===========

We developed this extension using `Visual Studio Code <https://code.visualstudio.com/>`__, the 
recommended tool on NVIDIA's Omniverse official documentation. To setup Visual Studio Code with hot-reloading features, follow
the additional documentation pages:

* `Isaac Sim VSCode support <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_standalone_python.html#isaac-sim-python-vscode>`__
* `Debugging with VSCode <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_advanced_python_debugging.html>`__

To enable a better developer experience when contributing or modifying this extension, and have access to features such as
autocomplete, we recommend linking the Pegasus Simulator repository to the current installation of ``ISAACSIM`` . For that, please
run the following script provided with the framework:

.. code:: bash

    ./link_app.sh --path $ISAACSIM_PATH

This script will create a symbolic link to ``ISAACSIM`` inside the repository. After this step, you can also launch the 
``ISAACSIM`` with the extension enabled (without using the GUI presented in Section :ref:`Installing the Pegasus Simulator`), by running:

.. code:: bash

    ./app/isaac-sim.sh --ext-folder extensions --enable pegasus.simulator

Code structure
--------------

This simulation framework is strucuture according to the following tree:

.. code:: bash

    PegasusSimulator:
    ├── .vscode
    ├── docs
    ├── examples
    ├── tools
    ├── extensions
    │   ├── pegasus.simulator
    │   │   ├── config
    │   │   ├── data
    │   │   ├── docs
    │   │   ├── setup.py
    │   │   ├── pegasus.simulator
    │   │   │   │   ├── __init__.py
    │   │   │   │   ├── params.py
    │   │   │   │   ├── extension.py
    │   │   │   │   ├── assets
    │   │   │   │   ├── logic
    │   │   │   │   ├── ui

The extensions directory contains the source code for the PegasusSimulator API and interactive GUI while the 
examples directory contains the a set of python scripts to launch standalone applications and pre-programed simulations.

As explained in NVIDIA's documentation, extensions are the standard way of developing on top of Isaac Sim and other Omniverse
tools. The core of our extension is developed in the ``logic`` and the ``ui`` modules. The ``logic`` API is exposed to the users
and is used both when operating the Pegasus Simulator in GUI widget/extension mode and via standalone python applications.
The ``ui`` API is **not exposed/documented** as it is basically defines the widget to call functionalities provided by 
the ``logic`` module.
