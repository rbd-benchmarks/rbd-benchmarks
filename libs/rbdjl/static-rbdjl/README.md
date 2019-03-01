# Calling RBD.jl static library from code compiled with a C++ compiler

Steps:

1. run `make` in this directory
2. `build/driver -u ../../description/urdf/atlas.urdf -c ../../csv/rbdjl/atlas_inputs.csv -f` (add `-f` flat to use a floating joint as the root joint)


## Variables:

* `CXX`: C++ compiler to use
* `JULIA`: location of Julia executable (default: `julia`)
* `SCALAR_TYPE`: which scalar type to use (default: `1`); options:
  1. `Float64` (`double`): double precision
  2. `Float32` (`float`): single precision

## Makefile targets:

* `all` (default): build Julia system image and driver
* `build/driver`: build the driver program
* `build/libstaticrbdjl.so`: build Julia system image
* `clean-driver`: remove driver program
* `clean`: remove entire `build` directory

## Notes:

* OSX support not yet working completely

