Contributing
============

The Pegasus Simulator is an open-source effort, started by me, Marcelo Jacinto in January/2023. It is a tool that was 
created with the original purpose of serving my Ph.D. workplan for the next 4 years, which means that you can expect 
this repository to be mantained by me directly, hopefully until 2027. 

With that said, it is very likely that you will stumble upon bugs on the code or missing features. Information on how 
to contribute with code, documentation or suggestions for the project roadmap can be found in the following sections.

Issues, Bug Reporting and Feature Requests
------------------------------------------

- **Bug reports:** Report bugs you find in the Github Issue tab.

- **Feature requests:** Suggest new features you would like to see in the Github Discussions tab.

- **Code contributions:** Submit a Github Pull Request (read the next section).

Branch and Version Model
------------------------

This project uses a two-branch Git model:

- **main:** By default points to the latest stable tag version of the project. 
- **dev:** Corresponds to an unstable versions of the code that are not well tested yet.

In this project, we avoid performing merges and give preference to a fork/pull-request structure. All code contributions 
have to be made under the permissive BSD 3-clause license and all code must not impose any further constraints on the use.

Contributing with Code
----------------------

Please follow these steps to contribute with code:

1. Create an issue in Github issue tab to discuss new changes or additions to the code.
2. `Fork <https://docs.github.com/en/get-started/quickstart/fork-a-repo>`__ the repository.
3. `Create a new branch <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-and-deleting-branches-within-your-repository>`__ for your changes.
4. Make your changes.
5. Run pre-commit on all files to make sure the code is well formated (check :ref:`Code Style` section).
6. Commit the changes following the guide in the :ref:`Commit Messages` section.
7. Update the documentation accordingly (check :ref:`Contributing with Documentation` section).
8. Push your changes to your forked repository.
9. `Submit a pull request <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork>`__ to the main branch of this project.
10. Ensure all the checks on the pull request are successful.

After sending a pull request, the developer team will review your code, provide feedback.

.. note::
   Ensure that your code is well-formatted, documented and working.

Commit Messages
~~~~~~~~~~~~~~~

Each commit message should be short and provide a good description of what is being changed or added to the code. As such, 
we suggest that every commit message start by: 

   * ``feat``: For new features.
   * ``fix``: For bug fixes.
   * ``rem``: For removing code or features.
   * ``doc``: For adding or changing documentation.
   * ``chore``: When none of the above are a good fit.

Here is an example of a "good" commit:

   .. code:: bash
      
      git commit -m "feat: new vehicle thurster dynamics"

.. note::
   We do not enforce strictly this commit policy, but it is highly recommended.

Pull Requests
~~~~~~~~~~~~~

The description of the Pull Request should include:

- An overview of what is adding, changing or removing; enough to understand the broad purpose of the code
- Links to related issues, supporting information or research papers (if useful).
- Information about what code testing has been conducted.

Code Style
~~~~~~~~~~

The inline code documentation follows the `Google Style Guides <https://google.github.io/styleguide/pyguide.html>`__ while the Python code follows the `PEP guidelines <https://peps.python.org/pep-0008/>`__. We use 
the `pre-commit <https://pre-commit.com/>`__ tool tools for maintaining code quality and consistency over the codebase. 
You can install ``pre-commit`` by running:

   .. code:: bash

      pip install pre-commit

If you do not want to polute your python environment, please use 
`venv <https://docs.python.org/3/library/venv.html>`__ or `conda <https://docs.conda.io/en/latest/>`__. 

To run ``pre-commit`` over the entire repository, execute:

   .. code:: bash

      pre-commit run --all-files

Contributing with Documentation
-------------------------------

I know, everyone hates to write documentation - its boring... but it is needed. That's why we tried
to make it easy to contribute to it. 

All the source files for the documentation are located in the ``docs`` directory. The documentation is written in 
`reStructuredText <https://www.sphinx-doc.org/en/master/>`__ format. We use Sphinx with the 
`Read the Docs Theme <https://readthedocs.org/projects/sphinx/>`__ for generating the documentation. Sending a pull 
request for the documentation is the same as sending a pull request for the codebase. Please follow the steps 
mentioned in the :ref:`Contributing with Code` section. 

To build the documentation, you need to install a few python 
dependencies. If you do not want to polute your python environment, please use 
`venv <https://docs.python.org/3/library/venv.html>`__ or `conda <https://docs.conda.io/en/latest/>`__.

To generate the html documentation, execute the following commands:

1. Enter the ``docs`` directory.

   .. code:: bash

     # (relative to the root of the repository)
     cd docs

2. Install the python dependencies.

   .. code:: bash

     pip install -r requirements.txt

3. Build the documentation.

   .. code:: bash

     make html

4. Open the documentation in a browser.

   .. code:: bash

     xdg-open _build/html/index.html

Contributing with Assets
------------------------

Creating 3D models is an hard and time consuming task. We encourage people to share models that they feel will be usefull
for the community, as long as:

1. The assets are appropriately licensed.
2. They can be distributed in an open-source repository.

.. note::

   Currently, we still do not have a standard approach for submitting open-source assets to be incorporated into Pegasus Simulator,
   but a possible solution in the future might lie either on hosting small sized ones on this repository and large
   worlds in a nucleus server. If you have a great idea regarding this subject, share it with us on the Github Issues tab!

Sponsor the project
-------------------

If you want to be a part of this project, or sponsor my work with some graphics cards, jetson developer boards and other development
material, please reach out to me directly at ``marcelo.jacinto@tecnico.ulisboa.pt``.

Current sponsors:

- Dynamics Systems and Ocean Robotics (DSOR) group of the Institute for Systems and Robotics (ISR), a research unit of the Laboratory of Robotics and Engineering Systems (LARSyS).
- Instituto Superior Técnico, Universidade de Lisboa

The work developed by Marcelo Jacinto and João Pinto was supported by Ph.D. grants funded by Fundação para as Ciências e Tecnologias (FCT).

.. raw:: html

   <p float="left" align="center">
      <img src="../../_static/dsor_logo.png" width="90" align="center" />
      <img src="../../_static/logo_isr.png" width="200" align="center"/> 
      <img src="../../_static/larsys_logo.png" width="200" align="center"/> 
      <img src="../../_static/ist_logo.png" width="200" align="center"/> 
      <img src="../../_static/logo_fct.png" width="200" align="center"/> 
   </p>
