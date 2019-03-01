#!/bin/bash
set -e
set -x
set -v

# Build Julia from source to avoid libc version issues.
# echo Installing Julia
# JULIA_BUILD_DIR=/tmp/julia-build
# JULIA_INSTALL_DIR=~/julia
# mkdir -p $JULIA_BUILD_DIR
# pushd $JULIA_BUILD_DIR
# git clone https://github.com/JuliaLang/julia.git
# pushd julia
# git checkout v1.1.0
# echo prefix=$JULIA_INSTALL_DIR > Make.user
# make -j 4
# make install
# export JULIA=$JULIA_INSTALL_DIR/julia
# popd
# popd

JULIA_INSTALL_DIR=~/julia
mkdir -p $JULIA_INSTALL_DIR
pushd $JULIA_INSTALL_DIR
wget https://julialang-s3.julialang.org/bin/linux/x64/1.1/julia-1.1.0-linux-x86_64.tar.gz
tar -xf julia-1.1.0-linux-x86_64.tar.gz
export JULIA=$JULIA_INSTALL_DIR/julia-1.1.0/bin/julia
popd
$JULIA -v

echo "Generating robot description files"
pushd description
make
popd

echo "Installing libraries"
pushd libs
make -j 4
popd

echo "Installing likwid"
pushd likwid
make -j 4
sudo make install
popd

# TODO
echo "Generating benchmark inputs"
pushd input_data_gen
make
popd

echo "Running experiments"
pushd experiments
./run_all_time.sh
./run_all_terr.sh
# ./run_all_likwid.sh # TODO
popd
