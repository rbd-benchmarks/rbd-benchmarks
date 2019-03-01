#!/bin/sh
# run_robcogen_likwid.sh
# Usage: ./run_robcogen_likwid.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
echo "Running LIKWID experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) RobCoGen:"
   cd $RBD_BENCHMARKS_PATH/testbenches/robcogen/likwid_robcogen
   for DYN_ALG in rnea crba aba
   do
      for EVENT_SET in branch l2cache mix mstall flopsdp
      do
         for ROBOT_MODEL in iiwa hyq atlas
         do
            echo "RobCoGen: $RESULTS, $DYN_ALG, $EVENT_SET, $ROBOT_MODEL"
            make clean
            make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG ROB=$ROBOT_MODEL EVENTS=$EVENT_SET
            ./likwid_robcogen
         done
      done
   done
cd $RBD_BENCHMARKS_PATH/experiments/robcogen
done
echo "Done!"
