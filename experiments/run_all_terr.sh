#!/bin/sh
# run_all_terr.sh
# Usage: ./run_all_terr.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
PINOCCHIO_INSTALL_PATH=$RBD_BENCHMARKS_PATH/libs/pinocchio/install
export PKG_CONFIG_PATH=$PINOCCHIO_INSTALL_PATH/lib/pkgconfig:$PINOCCHIO_INSTALL_PATH/share/pkgconfig:$PKG_CONFIG_PATH
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
   echo "($DATA_NUM) RBD.jl:"
   cd $RBD_BENCHMARKS_PATH/testbenches/rbdjl/terr_rbdjl
   for DYN_ALG in rnea crba aba
   do
      make clean
      make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG
      for ROBOT_MODEL in iiwa hyq atlas
      do
         echo "RBD.jl: $RESULTS, $DYN_ALG, $ROBOT_MODEL"
         if   [ "$ROBOT_MODEL" = "iiwa" ]
         then
            ./terr_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/iiwa.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/iiwa_inputs.csv
         elif [ "$ROBOT_MODEL" = "hyq" ]
         then
            ./terr_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/hyq.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/hyq_inputs.csv -f
         elif [ "$ROBOT_MODEL" = "atlas" ]
         then
            ./terr_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/atlas.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/atlas_inputs.csv -f
         else
            echo "Unrecognized robot model!"
         fi
      done
   done
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
cd $RBD_BENCHMARKS_PATH/experiments
done
echo "Done!"
