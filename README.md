# Pegasus Simulator

![IsaacSim 2022.2.0](https://img.shields.io/badge/IsaacSim-2022.2.0-brightgreen.svg)
![PX4-Autopilot 1.13.2](https://img.shields.io/badge/PX4--Autopilot-1.13.2-brightgreen.svg)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04LTS-brightgreen.svg)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg)

**Pegasus Simulator** is a framework built on top of [NVIDIA
Omniverse](https://docs.omniverse.nvidia.com/) and [Isaac
Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html). It is designed to provide an easy, yet powerfull way of simulating the dynamics of multirotors vehicles. It provides a simulation interface for PX4 integration as well as custom python control interface.

<p align = "center">
<img src="docs/_static/pegasus_cover.png" width=500 />
</p>

## Citation

Please cite [this paper]() if you use this framework in your work:

```
@misc{jacinto2023pegasus,
	author = {Jacinto, Marcelo and Pinto, João and Pascoal, António and Cunha, Rita},
	title = {Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation},
	year = {2023},
	eprint = {},
}
```

## Developer Team

This simulation framework is an open-source effort, started by me, Marcelo Jacinto in January/2023. It is a tool that was created with the original purpose of serving my Ph.D. workplan for the next 4 years, which means that you can expect this repository to be mantained by me directly, hopefully until 2027.

* Project Founder
	* [Marcelo Jacinto](https://github.com/MarceloJacinto), under the supervision of <u>Prof. Rita Cunha</u> and <u>Prof. Antonio Pascoal</u> (IST/ISR-Lisbon)
* Architecture
    * [Marcelo Jacinto](https://github.com/MarceloJacinto)
	* [João Pinto](https://github.com/jschpinto)
* Multirotor Dynamic Simulation and Control
    * [Marcelo Jacinto](https://github.com/MarceloJacinto)
* Example Applications
	* [Marcelo Jacinto](https://github.com/MarceloJacinto)
	* [João Pinto](https://github.com/jschpinto)

## Project Roadmap

An high level project roadmap is available [here]().

## Support and Contributing

We welcom new contributions from the community to improve this work. Please check the [Contributing]() section in the documentation for the guidelines on how to help improve and support this project.

* Use [Discussions]() for discussing ideas, asking questions, and requests features.
* Use [Issues]() to track work in development, bugs and documentation issues.
* Use [Pull Requests]() to fix bugs or contribute directly with your own ideas, code, examples or improve documentation.

## Licenses

NVIDIA Isaac Sim is available freely under [individual license](https://www.nvidia.com/en-us/omniverse/download/). 

Pegasus Simulator is released under [BSD-3 License](LICENSE). The license files of its dependencies and assets are present in the [`docs/licenses`](docs/licenses) directory.

## Project Sponsors
- Dynamics Systems and Ocean Robotics (DSOR) group (Portugal), under Marcelo Jacinto's Ph.D. grant funded by FCT.

<p float="left" align="center">
  <img src="docs/_static/dsor_logo.png" width="90" align="center" />
  <img src="docs/_static/ist_logo.png" width="200" align="center"/> 
</p>