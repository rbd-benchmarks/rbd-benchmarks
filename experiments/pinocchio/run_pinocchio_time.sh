#!/bin/sh
# run_pinocchio_time.sh
# Usage: ./run_pinocchio_time.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
PINOCCHIO_PATH=$RBD_BENCHMARKS_PATH/libs/pinocchio/install
export PKG_CONFIG_PATH=$PINOCCHIO_PATH/lib/pkgconfig:$PINOCCHIO_PATH/share/pkgconfig:$PKG_CONFIG_PATH
echo "Running time experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) Pinocchio:"
   cd $RBD_BENCHMARKS_PATH/testbenches/pinocchio/time_pinocchio
   make clean
   make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH
   for ROBOT_MODEL in iiwa hyq atlas
   do
      echo "Pinocchio: $RESULTS, time, $ROBOT_MODEL"
      ./time_pinocchio $ROBOT_MODEL
   done
cd $RBD_BENCHMARKS_PATH/experiments/pinocchio
done
echo "Done!"
