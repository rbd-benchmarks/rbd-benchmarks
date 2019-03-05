## RBD-Benchmarks

[![Build Status](https://travis-ci.org/rbd-benchmarks/rbd-benchmarks.svg?branch=master)](https://travis-ci.org/rbd-benchmarks/rbd-benchmarks)

### Repository layout

* `description` contains robot models
* `experiments` contains scripts to run experiments
* `input_data_gen` contains scripts to generate input data
* `libs` contains the rbd libraries
* `testbenches` contains the testbench code used by the experiments

Optional/Generated Directories:

* `csv` contains generated input data in csv format
* `results` is a directory to store experiment results

### Setup

`rbd-benchmarks` is currently Linux-only, and uses [Travis](https://travis-ci.org/) for continuous integration. Travis is currently set up to ensure that the code compiles on Ubuntu Xenial Xerus (16.04). `rbd-benchmarks` has also been user-tested on Ubuntu Bionic Beaver (18.04). Other platforms are untested and as such unlikely to work out of the box.

#### Dependencies

Please see our [Travis configuration file](.travis.yml) for an up-to-date list of dependencies. Note that while the version of Maxima (used by RobCoGen) available through `apt` in Ubuntu 16.04 is not sufficiently up-to-date, the version that ships with Ubuntu 18.04 is.

In addition, you will need Julia 1.1, both to run benchmarks for RigidBodyDynamics.jl, and to generate robot description files and input test data. Binaries for Julia can be obtained from https://julialang.org/downloads/. After decompressing the archive, either:

* set the `JULIA` environment variable to point to the `julia` executable: `export JULIA=/path/to/julia/julia`
* add the directory containing the `julia` executable to the `PATH`: `export PATH=/path/to/julia/:$PATH`

Finally, the C/C++ compiler to use may be specified by setting the `CC` and `CXX` environment variables. By default, the system C/C++ compiler will be used, which may be too old on Ubuntu 16.04 (symptom: linking errors while running `make` in `libs/rbdjl`).

#### Submodule initialization

After cloning the repository, please run

```bash
git submodule update --init --recursive
```

to initialize Git submodules.

#### Generating model description files

To generate URDF and KINDSL files according to the process described in the paper, run

```bash
cd description
make
cd ..
```

If this errors, please ensure that submodules have been initialized (previous step).

#### Building libraries

To download and install the rigid body dynamics libraries under test (locally), run

```bash
cd libs
make # or make -j 4 to build the libraries in parallel with four make workers
cd ..
```

#### Generating random input data

To generate the `csv` directory containing `.csv` files with library-specific random input data (with sample `i` for each library representing the same physical state / torque input), run

```bash
cd input_data_gen
make
cd ..
```

#### Installing likwid

This step is optional, and only required for the CPU instruction mix results.

To install the [likwid](https://github.com/RRZE-HPC/likwid) hardware performance counter tools, run

```bash
cd likwid
make -j 4
sudo make install
cd ..
```

Note that likwid will be installed locally, but `sudo` is required to `chown` the locally installed files to `root`, since these files [need access to model-specific register devicd files](https://github.com/RRZE-HPC/likwid/wiki/Build#setting-up-access-for-hardware-performance-monitoring).

Hardware performance counters differ per CPU architecture. Currently, only the Kaby Lake CPU architecture has been tested. Running the likwid experiments on other CPU architectures is likely to result in a program crash.

### Running experiments

To run timing experiments (average time) for all libraries, run

```bash
experiments/run_all_time.sh
```

To record run times for each sample in a `.csv` file for all libraries, run

```bash
experiments/run_all_terr.sh
```

To run likwid performance counter experiments, run

```bash
experiments/run_all_likwid.sh
```
