.. contents:: Exploring The Script Node
    :backlinks: entry
    :local:
    :depth: 1

What Is A Script Node?
----------------------

A script node is an implementation of a single node type for |omnigraph| that can be used to create many different
behaviors at runtime. Unlike most node types, whose behavior is hardcoded, each instantiation of a script node type
can have its own custom behavior, implemented as a Python script on the node.

Some of the best uses of a script node are, to make quick prototypes of functionality to validate concepts you are
trying out, and to write one-off nodes that are so simple that it is easier to write them yourself rather than to
try to find an equivalent node (e.g. write a node that computes the hexadecimal string equivalent of an integer).

Using The Script Node
---------------------

These are the three steps you need to add a customized script node to your |omnigraph|:

- Create a script node.
- Add the Python code required to implement the runtime behavior of the node.
- Add necessary attributes for its operation.

In practice steps 2 and 3 can be done in either order, though we think you will find it easier to first define your
function and then from that decide the list of attributes to be added.

.. important::

    The script node will not be operational until you have added the extra attributes required. In the graph editor
    this execution failure shows up as a red coloring on the node. Once you have added the attributes normal execution
    will resume.

The script node can be used either directly from the UI via the graph editor and property panel, or indirectly through
Python scripting, either in the script editor or in external files. The instructions below show both approaches -
choose the one that you feel most comfortable with.

Creation
++++++++

To start using the script node you must first create an |omnigraph| in which it can live, and then create an instance
of the script node in that graph.

.. tab-set::

    .. tab-item:: Graph Editor

       Open the :ref:`Action Graph Editor<ext_omni_graph_window_action>` using the *Visual Scripting* menu

       .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodeGraphMenu.png
            :alt: The graph being created

       Create a new |actiongraph| to hold the script node

       .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodeGraphCreate.png
            :alt: The graph being created

      Drag the script node icon from the navigation bar on the left onto the newly created graph

       .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodeCreate.png
            :alt: The Script Node being added to a graph

      Once you have created a script node it will look something like this in your graph editor. The script node is
      suitable for use in any type of graph. In an |actiongraph| the node will have an *execIn* pin to trigger its
      execution and an *execOut* pin to trigger other nodes when its compute completes.

    .. tab-item:: Python Code

        The |controller| is the main class through which you can manipulate an |omnigraph| using a script file or
        the script editor. This script will create an |actiongraph| and add a script node to it.

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/tests/test_ascentnode.py
            :language: python
            :start-after: begin-script-node-boilerplate
            :end-before: end-script-node-boilerplate

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/tests/test_ascentnode.py
            :language: python
            :dedent: 8
            :start-after: begin-script-node-creation
            :end-before: end-script-node-creation

Writing A Compute Function
++++++++++++++++++++++++++

The actual input from the script node can come from one of two sources - a string set directly on the node, or an
external file containing the script. These are defined using shared attributes, present in all script nodes. The
extra attributes you will use as part of your computation will be added later.

Contents Of The Script
~~~~~~~~~~~~~~~~~~~~~~

The script node creates a limited override of the API accessible from :py:class:`omni.graph.core.NodeType`.
The following callback functions can be defined in the script, and will be used as the runtime implementations of
the equivalent functions on the node type API.

- ``compute(db)``: called every time the node computes (should always be defined).
- ``setup(db)``: called before compute the first time, or after the reset attribute value is set.
- ``cleanup(db)``: called when the node is deleted or the reset attribute value is set.

*db:* :py:class:`omni.graph.core.Database` is the node interface where attributes are exposed like ``db.inputs.foo``.
This includes the predefined attributes used by the script node as described below, as well as any dynamic attributes
added to a script node instance by the user. The predefined functions ``db.log_error`` or ``db.log_warning`` should be
used to report problems in the node's computation.

In addition, for convenience the :py:mod:`omni.graph.core` module is imported under the variable named ``og``.

`import` statements, function/class definitions, and global variables may be placed,
outside of the callbacks, as you would in any other Python module definition.

Variables may be added to the ``db.per_instance_state`` state object for persistence across nodes that are instanced
in more than one graph. See how the
:ref:`sample snippet for the Fibonacci function <omni_graph_script_node_samples>` makes use of this feature to
walk through the values in the Fibonacci sequence on each successive evaluation.

Overriding the ``db.setup(db)`` and ``db.cleanup(db)`` functions can be used to let your script node define values
that will be used through multiple evaluations, but which you do not wish to persist when the script node itself is
deleted. See how the
:ref:`sample snippet for the Controller function <omni_graph_script_node_samples>` makes use of this feature to
initialize and clean up the USD stage for a script node that is responsible for creating cubes.

.. note::

    The `setup` function corresponds to the :py:meth:`omni.graph.core.NodeType.initialize` function on the node type
    and the `cleanup` function corresponds to the :py:meth:`omni.graph.core.NodeType.release` function. The reason they
    are different is that they will also be called when the script node is reset whereas the API functions only get
    called when the node is created and destroyed.

All of the attribute values you get back from calling the ``db.inputs`` or ``db.outputs`` properties have a specific
data type based on their attribute type.
You can find a description of all of the data types returned from the database for the supported attribute types
by looking through the :ref:`data type descriptions<omnigraph_data_types>`.

.. _omni_graph_script_node_as_text:

Setting The Script With Text
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The attribute `inputs:script` is a simple text string which will be later translated into Python.

.. tab-set::

    .. tab-item:: Graph Editor

        After creation of the script node it should be selected and its properties should be visible in the
        property panel. If you don't have the property panel visible you can turn it on with this menu

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodePropertyPanelMenu.png
            :alt: The menu entry to turn on the property panel

        This is what the property panel for your script node will look like on creation. Notice how the script
        field has been pre-populated with some placeholders for the functions you are allowed to write as well as some
        instructions on what the script can contain.

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodePropertyPanel.png
            :alt: The graph being created

        Here is the full text of the instructions:

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/_impl/ascentnode_example_scripts.py
            :language: python
            :start-after: Title = "Default Script"
            :end-before: # # # DELIMITER # # #

        Ignoring the `setup(db)` and `cleanup(db)` functions for now copy-paste this simple node type definition string
        and replace the **Script** text field with it.

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/tests/test_ascentnode.py
            :language: python
            :dedent: 8
            :start-after: begin-script-node-script
            :end-before: end-script-node-script

    .. tab-item:: Python Code

        The attribute values required to point the script node at a file can be set through the |controller|.
        Here is an example of a simple script that defines a script node that will output a boolean indicating
        whether the first input is greater than the second input.

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/tests/test_ascentnode.py
            :language: python
            :dedent: 8
            :start-after: begin-script-node-set-to-script
            :end-before: end-script-node-set-to-script

..  tip::

    As script nodes do not have unique node type definitions it is always a good idea to add Python docstrings as
    documentation as a reminder of exactly what the node does.

Now that you have a script defined you can skip ahead to :ref:`omni_graph_script_node_adding_attributes`.

Setting The Script With A File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to use an external file then you set the *Use Path* attribute to **True** and set the *Script Path*
attribute to be a string containing the path to the script file on disk. It can be an absolute path name, which will be
highly reliant on your file system configuration, or it can be relative to the USD edit layer so that the script can
be passed along with your USD file as a "sidecar" file.

.. tab-set::

    .. tab-item:: Graph Editor

        After creation of the script node it should be selected and its properties should be visible in the
        property panel. If you don't have the property panel visible you can turn it on with this menu

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodePropertyPanelMenu.png
            :alt: The menu entry to turn on the property panel

        This is what the property panel for your script node will look like on creation. Notice how the script
        field has been pre-populated with some placeholders for the functions you are allowed to write as well as some
        instructions on what the script can contain.

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodePropertyPanel.png
            :alt: The graph being created

        Now check the *Use Path* checkbox to tell the script node that it is getting its input from a file rather than
        the *Script* value above. Next set the *Script File Path* value to point to a file in which you have put your
        script. (See the :ref:`omni_graph_script_node_as_text` script section above for an example of what you
        might put into your file.) When you are done your property panel should look something like this.

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodeAsFile.png
            :alt: The property panel with a file script specified

    .. tab-item:: Python Code

        The attribute values required to point the script node at a file can be set through the |controller|.
        This example creates a temporary file with the same script as the example above and accesses it.

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/tests/test_ascentnode.py
            :language: python
            :dedent: 8
            :start-after: begin-script-node-set-to-file
            :end-before: end-script-node-set-to-file

.. _omni_graph_script_node_adding_attributes:

Adding Attributes
+++++++++++++++++

Since there is no .ogn description of the node your script node will rely on |dynamicattributes| to define the inputs
and outputs of the node. A dynamic attribute is just one that is not predefined by the node type. In the script
you have written above these appear as anything in your `compute()` functions accessed as **db.inputs.X** for input
attributes and **db.outputs.Y** for output attributes.

As the intent of your code is unknown (e.g. did you mean to add two integers or two arrays of points when you typed
*db.inputs.a + db.inputs.b*) you must manually add each of the attributes with the types you intend to use.

.. tab-set::

    .. tab-item:: Graph Editor

        In the script node property panel you will see a button labeled **Add Attribute...**. You will click on it
        once for each attribute that your `compute()` function requires; in this case it will be two inputs and one
        output.

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodeAddingAttributes.png
            :alt: Property panel for adding attributes

        This brings up a dialog where you can define your attributes. Here is what you will enter in order to define the
        first attribute as an integer value.

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodeAddInput.png
            :alt: An input being added to a script node in the property panel

        Repeat this for the other input *second_input* and then once again for the output attribute.

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodeAddOutput.png
            :alt: An output being added to a script node in the property panel

        Notice here that you must also click the **output** button to specify that the new attribute will be an output.
        As a rule of thumb, inputs are values that you read and outputs are values that you write.

        Once you have created a script node it will look something like this in your graph editor. The script node is
        suitable for use in any type of graph. In an |actiongraph| the node will have an *execIn* pin to trigger its
        execution and an *execOut* pin to trigger other nodes when its compute completes.

        .. image:: ../../../../../source/extensions/airlab.pegasus/python/nodes/images/AscentNodeFinalNode.png
            :alt: The Script Node in the editor after adding attributes

    .. tab-item:: Python Code

        The |controller| can also be used to add attributes to a node. This example makes our two inputs *integer*
        types and the output a *boolean* type. Note that this is a continuation of the previous script as it must
        appear inside the *TemporaryDirectory* context to work properly.

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/tests/test_ascentnode.py
            :language: python
            :dedent: 8
            :start-after: begin-script-node-add-attributes
            :end-before: end-script-node-add-attributes

.. note::

    You may have seen references to another type of attribute port beyond input and output. The *state* attribute port
    is just like an *output* port except that it is guaranteed to retain its value between executions of the
    `compute()` function. Use state attributes for temporarily caching information.

Tradeoffs: Script Node vs. Python Node
--------------------------------------

The script node, accessible when you load the extension :ref:`airlab.pegasus<ext_airlab.pegasus>`,
provides a generic node type inside |omnigraph|. It includes an input attribute that holds a Python script
encoded as a string. This string acts as the implementation of this node.

Although the syntax is slightly different from what you might find in a normal Python node
the benefit of the script node is that you do not have to write any external files, including any
.ogn definitions to implement the new node.

The downside is that, since the script node you write is not on disk, it is more difficult to share the implementation
with other users.

.. note::

    This is an important distinction. In simple terms, a **node type** is like a blueprint for making nodes. The
    blueprint can be used by scripts or by the graph editor to create as many nodes of the same type as you wish.
    A **node** is the actual thing created based on that blueprint. Think of it like a cookie cutter (node type)
    used to make cookies (nodes).

    The **script node type** then is a general template for creating nodes that run scripts. A **script node** is a
    specific cookie made using that template, having its unique attributes and a Python script to run. While everyone
    can use the same cookie cutter (script node type) to make cookies (nodes) using standard tools, to create a new,
    specific cookie (script node), you'd have to duplicate an existing one.

.. _omni_graph_script_node_samples:

Code Snippets: Pre-Packaged Code Samples
----------------------------------------

If you have been using the property panel for editing you may have noticed a button labeled **Code Snippets**. This
button accesses a drop-down menu that will populate your `compute()` function with working examples. You may have to
enable extra extensions to make them work (e.g. **omni.warp**), and you will definitely have to inspect the snippets
to see what types of attributes they are expecting as those must still be added manually by you.

.. tab-set::

    .. tab-item:: Compute Count

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/_impl/ascentnode_example_scripts.py
            :language: python
            :start-after: Title = "Compute Count"
            :end-before: # # # DELIMITER # # #

    .. tab-item:: Fibonacci

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/_impl/ascentnode_example_scripts.py
            :language: python
            :start-after: Title = "Fibonacci"
            :end-before: # # # DELIMITER # # #

    .. tab-item:: Controller

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/_impl/ascentnode_example_scripts.py
            :language: python
            :start-after: Title = "Controller"
            :end-before: # # # DELIMITER # # #

    .. tab-item:: Warp

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/_impl/ascentnode_example_scripts.py
            :language: python
            :start-after: Title = "Sine Deformer With Warp"
            :end-before: # # # DELIMITER # # #

    .. tab-item:: Callbacks

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/_impl/ascentnode_example_scripts.py
            :language: python
            :start-after: Title = "Value Changed Callbacks"
            :end-before: # # # DELIMITER # # #

    .. tab-item:: Timer

        .. literalinclude:: ../../../../../source/extensions/airlab.pegasus/python/_impl/ascentnode_example_scripts.py
            :language: python
            :start-after: Title = "Compute Timer"
