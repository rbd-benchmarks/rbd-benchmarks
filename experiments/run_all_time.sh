#!/bin/sh
# run_all_time.sh
# Usage: ./run_all_time.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
PINOCCHIO_INSTALL_PATH=$RBD_BENCHMARKS_PATH/libs/pinocchio/install
export PKG_CONFIG_PATH=$PINOCCHIO_INSTALL_PATH/lib/pkgconfig:$PINOCCHIO_INSTALL_PATH/share/pkgconfig:$PKG_CONFIG_PATH
echo "Running time experiments..."
for DATA_NUM in 0
do
   echo "($DATA_NUM) RBDL:"
   cd $RBD_BENCHMARKS_PATH/testbenches/rbdl/time_rbdl
   make clean
   make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH
   for ROBOT_MODEL in iiwa hyq atlas
   do
      echo "RBDL: $RESULTS, time, $ROBOT_MODEL"
      ./time_rbdl $ROBOT_MODEL
   done
   echo "($DATA_NUM) Pinocchio:"
   cd $RBD_BENCHMARKS_PATH/testbenches/pinocchio/time_pinocchio
   make clean
   make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH
   for ROBOT_MODEL in iiwa hyq atlas
   do
      echo "Pinocchio: $RESULTS, time, $ROBOT_MODEL"
      ./time_pinocchio $ROBOT_MODEL
   done
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
cd $RBD_BENCHMARKS_PATH/experiments
done
echo "Done!"
