#!/bin/sh
# run_rbdjl_time.sh
# Usage: ./run_rbdjl_time.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/../..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
echo "Running time experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) RBD.jl:"
   cd $RBD_BENCHMARKS_PATH/testbenches/rbdjl/time_rbdjl
   make clean
   make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH
   for ROBOT_MODEL in iiwa hyq atlas
   do
      echo "RBD.jl: $RESULTS, time, $ROBOT_MODEL"
      if   [ "$ROBOT_MODEL" = "iiwa" ]
      then
         ./time_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/iiwa.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/iiwa_inputs.csv
      elif [ "$ROBOT_MODEL" = "hyq" ]
      then
         ./time_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/hyq.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/hyq_inputs.csv -f
      elif [ "$ROBOT_MODEL" = "atlas" ]
      then
         ./time_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/atlas.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/atlas_inputs.csv -f
      else
         echo "Unrecognized robot model!"
      fi
   done
cd $RBD_BENCHMARKS_PATH/experiments/rbdjl
done
echo "Done!"
