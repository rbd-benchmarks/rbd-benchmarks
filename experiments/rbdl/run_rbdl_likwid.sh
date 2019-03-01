#!/bin/sh
# run_rbdl_likwid.sh
# Usage: ./run_rbdl_likwid.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
echo "Running LIKWID experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) RBDL:"
   cd $RBD_BENCHMARKS_PATH/testbenches/rbdl/likwid_rbdl
   for DYN_ALG in rnea crba aba
   do
      for EVENT_SET in branch l2cache mix mstall flopsdp
      do
         make clean
         make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG EVENTS=$EVENT_SET
         for ROBOT_MODEL in iiwa hyq atlas
         do
            echo "RBDL: $RESULTS, $DYN_ALG, $EVENT_SET, $ROBOT_MODEL"
            ./likwid_rbdl $ROBOT_MODEL
         done
      done
   done
cd $RBD_BENCHMARKS_PATH/experiments/rbdl
done
echo "Done!"
