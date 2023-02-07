# Pegasus Simulator

Pegasus Isaac is an isaac-sim extension design to provide control engineers an easy way to simulate the dynamics of
multirotors vehicles. It provides a simulation interface for PX4 integration as well as ROS2.

## Contributing

The developers of the Pegasus simulator welcome any positive contributions and ideas from the robotics comunity to make
in order to allow this extension to mature. If you think you have a nice contribution to make or just a simple suggestion,
feel free to create bug reports, feature requests or open pull requests for direct code contributions. Please, check our
[contribution guidelines](https://TODO) so that we can all follow the
same standard!

## Acknowledgement

NVIDIA Isaac Sim is available freely under [individual license](https://www.nvidia.com/en-us/omniverse/download/).
For more information about its license terms, please check [here](https://docs.omniverse.nvidia.com/app_isaacsim/common/NVIDIA_Omniverse_License_Agreement.html#software-support-supplement).

Pegasus Simulator is released under [BSD-3 License](LICENSE).
The license files of its dependencies and assets are present in the [`docs/licenses`](docs/licenses) directory.

## Citation
Please cite [this paper](https://arxiv.org/abs/2301.04195) if you use this extension in your work:

```
@misc{jacinto2023pegasus,
	author = {Marcelo Jacinto and Rita Cunha},
	title = {Pegasus: A simulation extension dedicated to aerial vehicle applied to the IsaacSim},
	year = {2023},
	eprint = {},
}
```

## Building Documentation

We use [Sphinx](https://www.sphinx-doc.org/en/master/) with the [Read the Docs Theme](https://sphinx-rtd-theme.readthedocs.io/en/stable/) for maintaining the documentation.

> **Note:** To build the documentation, we recommend creating a virtual environment to avoid any conflicts with system installed dependencies.

Execute the following instructions to build the documentation (assumed from the top of the repository):

1. Install the dependencies for [Sphinx](https://www.sphinx-doc.org/en/master/):

    ```bash
    # enter the location where this readme exists
    cd docs
    # install dependencies
    pip install -r requirements.txt
    ```

2. Generate the documentation file via:

    ```bash
    # make the html version
    make html
    ```

3. The documentation is now available at `docs/_build/html/index.html`:

    ```bash
    # open on default browser
    open _build/html/index.html
    ```
