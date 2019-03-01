#!/bin/sh
# run_robcogen_terr.sh
# Usage: ./run_robcogen_terr.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
ROBCOGEN_INSTALL_PATH=$RBD_BENCHMARKS_PATH/libs/robcogen/install
echo "Running time error experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) RobCoGen:"
   cd $RBD_BENCHMARKS_PATH/testbenches/robcogen/terr_robcogen
   for DYN_ALG in rnea crba aba
   do
      for ROBOT_MODEL in iiwa hyq atlas
      do
         echo "RobCoGen: $RESULTS, $DYN_ALG, $ROBOT_MODEL"
         make clean
         make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG ROB=$ROBOT_MODEL
         ./terr_robcogen
      done
   done
cd $RBD_BENCHMARKS_PATH/experiments/robcogen
done
echo "Done!"
