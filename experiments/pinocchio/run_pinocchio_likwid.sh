#!/bin/sh
# run_pinocchio_likwid.sh
# Usage: ./run_pinocchio_likwid.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
PINOCCHIO_PATH=$RBD_BENCHMARKS_PATH/libs/pinocchio/install
export PKG_CONFIG_PATH=$PINOCCHIO_PATH/lib/pkgconfig:$PINOCCHIO_PATH/share/pkgconfig:$PKG_CONFIG_PATH
echo "Running LIKWID experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) Pinocchio:"
   cd $RBD_BENCHMARKS_PATH/testbenches/pinocchio/likwid_pinocchio
   for DYN_ALG in rnea crba aba
   do
      for EVENT_SET in branch l2cache mix mstall flopsdp
      do
         make clean
         make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG EVENTS=$EVENT_SET
         for ROBOT_MODEL in iiwa hyq atlas
         do
            echo "Pinocchio: $RESULTS, $DYN_ALG, $EVENT_SET, $ROBOT_MODEL"
            ./likwid_pinocchio $ROBOT_MODEL
         done
      done
   done
cd $RBD_BENCHMARKS_PATH/experiments/pinocchio
done
echo "Done!"
