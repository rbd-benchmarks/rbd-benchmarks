#!/bin/sh
# run_all_check.sh
# Usage: ./run_all_check.sh

HERE=$PWD

RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results_check

ROBCOGEN_INSTALL_PATH=$RBD_BENCHMARKS_PATH/libs/robcogen/install

PINOCCHIO_PATH=$RBD_BENCHMARKS_PATH/libs/pinocchio/install
export PKG_CONFIG_PATH=$PINOCCHIO_PATH/lib/pkgconfig:$PINOCCHIO_PATH/share/pkgconfig:$PKG_CONFIG_PATH

mkdir -p $RESULTS_PATH

for lib in rbdl rbdjl robcogen pinocchio
do
  echo "Check Results for " + $lib + "..."
  mkdir -p $RESULTS_PATH/$lib
  cd $RBD_BENCHMARKS_PATH/testbenches/$lib/check_$lib
  for rob in hyq iiwa atlas
  do
    for alg in rnea crba aba
    do
      make clean
      make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH ALG=$alg ROB=$rob
      ./check_$lib > $RESULTS_PATH/$lib/$lib.$rob.$alg.out
    done
  done
done
cd $HERE
echo "Done!"
