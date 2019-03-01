#!/bin/sh
# run_all_likwid.sh
# Usage: ./run_all_likwid.sh
RBD_BENCHMARKS_PATH=`realpath $(dirname "$0")/..`
RESULTS_PATH=$RBD_BENCHMARKS_PATH/results
PINOCCHIO_INSTALL_PATH=$RBD_BENCHMARKS_PATH/libs/pinocchio/install
export PKG_CONFIG_PATH=$PINOCCHIO_INSTALL_PATH/lib/pkgconfig:$PINOCCHIO_INSTALL_PATH/share/pkgconfig:$PKG_CONFIG_PATH
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
   echo "($DATA_NUM) Pinocchio:"
   cd $RBD_BENCHMARKS_PATH/testbenches/pinocchio/likwid_pinocchio
   for DYN_ALG in rnea crba aba
   do
      for EVENT_SET in branch l2cache mix mstall flopsdp
      do
         make clean
         make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG EVENTS=$EVENT_SET
         for ROBOT_MODEL in iiwa hyq atlas
         do
            echo "Pinocchio: $RESULTS, $DYN_ALG, $EVENT_SET, $ROBOT_MODEL"
            ./likwid_pinocchio $ROBOT_MODEL
         done
      done
   done
   echo "($DATA_NUM) RBD.jl:"
   cd $RBD_BENCHMARKS_PATH/testbenches/rbdjl/likwid_rbdjl
   for DYN_ALG in rnea crba aba
   do
      for EVENT_SET in branch l2cache mix mstall flopsdp
      do
         make clean
         make RBD_BENCHMARKS=$RBD_BENCHMARKS_PATH RESULTS=$RESULTS_PATH ALG=$DYN_ALG EVENTS=$EVENT_SET
         for ROBOT_MODEL in iiwa hyq atlas
         do
            echo "RBD.jl: $RESULTS, $DYN_ALG, $EVENT_SET, $ROBOT_MODEL"
            if   [ "$ROBOT_MODEL" = "iiwa" ]
            then
               ./likwid_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/iiwa.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/iiwa_inputs.csv
            elif [ "$ROBOT_MODEL" = "hyq" ]
            then
               ./likwid_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/hyq.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/hyq_inputs.csv -f
            elif [ "$ROBOT_MODEL" = "atlas" ]
            then
               ./likwid_rbdjl -u $RBD_BENCHMARKS_PATH/description/urdf/atlas.urdf -c $RBD_BENCHMARKS_PATH/csv/rbdjl/atlas_inputs.csv -f
            else
               echo "Unrecognized robot model!"
            fi
         done
      done
   done
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
cd $RBD_BENCHMARKS_PATH/experiments
done
echo "Done!"
