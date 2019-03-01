#!/bin/sh
# run_robcogen_time.sh
# Usage: ./run_robcogen_time.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
ROBCOGEN_INSTALL_PATH=$RBD_BENCHMARKS_PATH/libs/robcogen/install
echo "Running time experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) RobCoGen:"
   cd $RBD_BENCHMARKS_PATH/testbenches/robcogen/time_robcogen
   for ALG_NAME in rnea crba aba
   do
      for ROBOT_MODEL in iiwa hyq atlas
      do
         echo "RobCoGen: $RESULTS, time, $ALG_NAME, $ROBOT_MODEL"
         make clean
         make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$ALG_NAME ROB=$ROBOT_MODEL
         ./time_robcogen
      done
   done
cd $RBD_BENCHMARKS_PATH/experiments/robcogen
done
echo "Done!"
