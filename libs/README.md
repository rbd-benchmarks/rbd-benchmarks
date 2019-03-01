# Downloading and Installing Libraries

Each subdirectory contains a library. Each library may be built either using the `Makefile` in the library's subdirectory, or using the `Makefile` in this directory, which recursively calls the library's `Makefile` targets.

## Building a single library

To build a single library, e.g. `rbdl`, use

```bash
make rbdl
```

from this directory. This will build the default target (`all`) in `rbdl/Makefile`. Other targets may be built using the `TARGET` argument, e.g.:

```bash
make rbdl TARGET=clean
```

## Building all libraries

To build all libraries in one go, simply use

```bash
make
```

and to specify the C++ compiler to be used for all libraries and build in parallel using 4 workers, use e.g.

```bash
CXX=clang++ make -j 4
```

To clean all libraries, use

```bash
make clean
```
