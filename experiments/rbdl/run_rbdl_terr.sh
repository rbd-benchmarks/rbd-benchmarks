#!/bin/sh
# run_rbdl_terr.sh
# Usage: ./run_rbdl_terr.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
echo "Running time error experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) RBDL:"
   cd $RBD_BENCHMARKS_PATH/testbenches/rbdl/terr_rbdl
   for DYN_ALG in rnea crba aba
   do
      make clean
      make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG
      for ROBOT_MODEL in iiwa hyq atlas
      do
         echo "RBDL: $RESULTS, $DYN_ALG, $ROBOT_MODEL"
         ./terr_rbdl $ROBOT_MODEL
      done
   done
cd $RBD_BENCHMARKS_PATH/experiments/rbdl
done
echo "Done!"
