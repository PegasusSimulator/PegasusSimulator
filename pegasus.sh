#!/bin/bash

# This instalation file is based on the original version provided in: 
# https://github.com/NVIDIA-Omniverse/Orbit
# Copyright (c) 2022, ETH Zurich
# Copyright (c) 2022, University of Toronto
# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES
# ------------------------------------------
# Share some love with those champs :)

# Exits if error occurs
set -e

# Set tab-spaces
tabs 4

# Get source directory
export PEGASUS_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Get the directory of isaac sim
if [ ! -z ${ISAACSIM_PATH} ]; then
    export ISAACSIM_PATH=${HOME}/.local/share/ov/pkg/isaac_sim-2022.2.*/python.sh
fi

#==
# Helper functions
#==

# extract the python from isaacsim
extract_isaacsim_python() {
    # Check if IsaacSim directory manually specified
    # Note: for manually build isaacsim, this: _build/linux-x86_64/release
    if [ ! -z ${ISAACSIM_PATH} ];
    then
        # Use local build
        build_path=${ISAACSIM_PATH}
    else
        # Use TeamCity build
        build_path=${PEGASUS_PATH}/_isaac_sim
    fi
    # python executable to use
    local python_exe=${build_path}/python.sh
    # check if there is a python path available
    if [ ! -f "${python_exe}" ]; then
      echo "[ERROR] No python executable found at path: ${build_path}" >&2
      exit 1
    fi
    # return the result
    echo ${python_exe}
}

# extract the simulator exe from isaacsim
extract_isaacsim_exe() {
    # Check if IsaacSim directory manually specified
    # Note: for manually build isaacsim, this: _build/linux-x86_64/release
    if [ ! -z ${ISAACSIM_PATH} ];
    then
        # Use local build
        build_path=${ISAACSIM_PATH}
    else
        # Use TeamCity build
        build_path=${PEGASUS_PATH}/_isaac_sim
    fi
    # python executable to use
    local isaacsim_exe=${build_path}/isaac-sim.sh
    # check if there is a python path available
    if [ ! -f "${isaacsim_exe}" ]; then
      echo "[ERROR] No isaac-sim executable found at path: ${build_path}" >&2
      exit 1
    fi
    # return the result
    echo ${isaacsim_exe}
}

# check if input directory is a python extension and install the module
install_pegasus_extension() {
    # retrieve the python executable
    python_exe=$(extract_isaacsim_python)
    # if the directory contains setup.py then install the python module
    if [ -f "$1/setup.py" ];
    then
        echo -e "\t module: $1"
        ${python_exe} -m pip install --editable $1
    fi
}

# print the usage description
print_help () {
    echo -e "\nusage: $(basename "$0") [-h] [-i] [-e] [-p] [-s] -- Utility to manage the Pegasus Simulator Extensions."
    echo -e "\noptional arguments:"
    echo -e "\t-h, --help       Display the help content."
    echo -e "\t-i, --install    Install the extensions Pegasus Simulator."
    echo -e "\n" >&2
}


#==
# Main
#==

# check argument provided
if [ -z "$*" ]; then
    echo "[Error] No arguments provided." >&2;
    print_help
    exit 1
fi

# pass the arguments
while [[ $# -gt 0 ]]; do
    # read the key
    case "$1" in
        # install the python packages pegasus_simulator/extensions directory
        -i|--install)
            echo "[INFO] Installing extensions..."
            # recursively look into directories and install them
            # this does not check dependencies between extensions
            export -f extract_isaacsim_python
            export -f install_pegasus_extension
            # initialize git hooks
            pip install pre-commit
            # source directory
            find -L "${PEGASUS_PATH}/source/extensions" -mindepth 1 -maxdepth 1 -type d -exec bash -c 'install_pegasus_extension "{}"' \;
            # unset local variables
            unset install_pegasus_extension
            shift # past argument
            ;;
        # run the formatter over the repository
        -f|--format)
            echo "[INFO] Formatting the repository..."
            pre-commit run --all-files
            shift # past argument
            # exit neatly
            break
            ;;
        -h|--help)
            print_help
            exit 1
            ;;
        *) # unknown option
            echo "[Error] Invalid argument provided: $1"
            print_help
            exit 1
            ;;
    esac
done
