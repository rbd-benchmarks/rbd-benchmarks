#!/bin/sh
# run_pinocchio_terr.sh
# Usage: ./run_pinocchio_terr.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
PINOCCHIO_PATH=$RBD_BENCHMARKS_PATH/libs/pinocchio/install
export PKG_CONFIG_PATH=$PINOCCHIO_PATH/lib/pkgconfig:$PINOCCHIO_PATH/share/pkgconfig:$PKG_CONFIG_PATH
echo "Running time error experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) Pinocchio:"
   cd $RBD_BENCHMARKS_PATH/testbenches/pinocchio/terr_pinocchio
   for DYN_ALG in rnea crba aba
   do
      make clean
      make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG
      for ROBOT_MODEL in iiwa hyq atlas
      do
         echo "Pinocchio: $RESULTS, $DYN_ALG, $ROBOT_MODEL"
         ./terr_pinocchio $ROBOT_MODEL
      done
   done
cd $RBD_BENCHMARKS_PATH/experiments/pinocchio
done
echo "Done!"
