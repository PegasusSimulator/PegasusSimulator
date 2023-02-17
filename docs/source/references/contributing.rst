Contributing
============

Information on how to contribute with code, documentation or suggestions for the project roadmap can be found in the following sections.

Issues, Bug Reporting and Feature Requests
------------------------------------------

- **Bug reports:** Report bugs you find in the Github Issue tab.

- **Feature requests:** Suggest new features you would like to see in the Github Discussions tab.

- **Code contributions:** Submit a Github Pull Request (read the next section).


Contributing with Code
----------------------

Please follow these steps to contribute with code:

1. Create an issue in Github issue tab to discuss new changes or additions to the code.
2. `Fork <https://docs.github.com/en/get-started/quickstart/fork-a-repo>`__ the repository.
3. `Create a new branch <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-and-deleting-branches-within-your-repository>`__ for your changes.
4. Make your changes and commit them.
5. Update the documentation accordingly.
6. Push your changes to your forked repository.
7. Submit a pull request to the main branch of this project.
8. Ensure all the checks on the pull request are successful.

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

Here is an example of a "good" commit:

   .. code:: bash
      
      git commit -m "feat: new vehicle thurster dynamics"

.. note::
   We do not enforce strictly this commit policy, but it is highly recommended.

Code Style
~~~~~~~~~~

Pull Requests
~~~~~~~~~~~~~

Branch and Version Model
~~~~~~~~~~~~~~~~~~~~~~~~

Contributing with Documentation
-------------------------------

I know, everyone hates to write documentation - its boring... but it is needed. That's why we tried
to make it easy to contribute to it. 

All the source files for the documentation are located in the ``docs`` directory. The documentation is written in 
reStructuredText format. We use Sphinx with the Read the Docs Theme for generating the documentation.

Sending a pull request for the documentation is the same as sending a pull request for the codebase. Please follow the steps mentioned in the Contributing Code section.

To build the documentation, we recommend creating a virtual environment to install the dependencies. This can also be a conda environment.

Contributing with Assets
------------------------

Contributing to repository
--------------------------

The Pegasus Simulator is an open-source effort, started by me, Marcelo Jacinto in January/2023. It is a tool that was 
created with the original purpose of serving my Ph.D. workplan for the next 4 years, which means that you can expect 
this repository to be mantained by me directly, hopefully until 2027. 

With that said, it is very likely that you will stumble upon bugs on the code or missing features. If you feel that there is
some critical feature missing and want to contribute to this project, suggest a new feature or just improve the documentation,
please check and use the issues page on github.

Sponsor the project
-------------------

If you want to be a part of this project, or sponsor my work with some graphics cards, jetson developer boards and other development
material, please reach out to me directly at ``marcelo.jacinto@tecnico.ulisboa.pt``.

At the moment, this project as it stands only has one direct sponsor:

- Dynamics Systems and Ocean Robotics (DSOR) group (Portugal), under Marcelo Jacinto's Ph.D. grant funded by FCT.

.. raw:: html

   <p float="left" align="center">
   <img src="../../_static/dsor_logo.png" alt="DSOR group at ISR-Lisbon" width="90" align="center" />
   <img src="../../_static/ist_logo.png" alt="Instituto Superior Técnico" width="200" align="center"/> 
   </p>
