#include <declarations.h>
#include <transforms.h>
#include <inertia_properties.h>
#include <inverse_dynamics.h>
#include <forward_dynamics.h>
#include <jsim.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "csv_reader.h"
#include "tictoc_timer.h"

#define VSIZE 6

//#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth)

#define NBT (1000*100)

using namespace iit::ROBOT_NAMESPACE;

int main(void) {

  // Set up timer
  TicToc timer(TicToc::NS);

  // Set robot model
  std::string robot_model = ROBOT_MODEL;
  std::cout << "Robot Model = " << robot_model << std::endl;

  // Import CSV inputs
  std::string input_filename = RBD_BENCHMARKS_DIR"/csv/robcogen/"+robot_model+"_inputs.csv";
  std::ifstream input_csv(input_filename.c_str());
  CSVRow row;
  input_csv >> row;

  std::vector<JointState>  qs     (NBT, JointState::Zero());
  std::vector<JointState>  qdots  (NBT, JointState::Zero());
  std::vector<JointState>  taus   (NBT, JointState::Zero());
  std::vector<JointState>  qddots (NBT, JointState::Zero());

#if FLOATING
  std::vector<iit::rbd::VelocityVector>     vs    (NBT, iit::rbd::VelocityVector::Zero());
  std::vector<iit::rbd::VelocityVector> vdots (NBT, iit::rbd::VelocityVector::Zero());
  iit::rbd::VelocityVector g = iit::rbd::VelocityVector::Zero();
  JointState jForces = JointState::Zero();
#endif

  int tot_q = JointSpaceDimension;

  std::cout << "qs, qdots = " << tot_q << ", " << tot_q << std::endl;

  int col, start_col;
  for(int i=0;i<NBT;++i)
  {
    input_csv >> row;
    start_col = 0;                    // 0
    for(int j=0;j<tot_q;++j)
    {
      col = start_col+j;
      qs[i][j] = atof(row[col].c_str());
    }
    start_col += tot_q;                // 1xQ

#if FLOATING
    for(int j=0;j<VSIZE;++j)
    {
      col = start_col+j;
      vs[i][j] = atof(row[col].c_str());
    }
    start_col += VSIZE;        // 1xQ + 6
#endif

    for(int j=0;j<tot_q;++j)
    {
      col = start_col+j;
      qdots[i][j] = atof(row[col].c_str());
    }
    start_col += tot_q;      // 2xQ + 6

#if FLOATING    
    for(int j=0;j<VSIZE;++j)
    {
      col = start_col+j;
      vdots[i][j] = atof(row[col].c_str());
    }
    start_col += VSIZE;    // 2xQ + 2x6
#endif

    for(int j=0;j<tot_q;++j)
    {
      col = start_col+j;
      qddots[i][j] = atof(row[col].c_str());
    }

#if FLOATING
    start_col += VSIZE;
#endif

    start_col += tot_q;   // 3xQ + 3x6
    for(int j=0;j<tot_q;++j)
    {
      col = start_col+j;
      taus[i][j] = atof(row[col].c_str());
    }

  }

  ForceTransforms ftransforms;
  MotionTransforms transforms;
  dyn::InertiaProperties inertias;
  dyn::InverseDynamics invdyn(inertias, transforms);
  dyn::ForwardDynamics frwdyn(inertias, transforms);
  dyn::JSIM jsimdyn(inertias, ftransforms);

  // Initialize some output variables
  double time_data;

   std::cout << "--" << std::endl;

   // Write CSV outputs
   std::string algorithm_name = ALG_NAME;
   std::ofstream output_csv;
   std::string output_filename;
   std::stringstream num_inputs;
   num_inputs << NBT;
   output_filename = RESULTS_DIR"/terr_"+robot_model+"_"+num_inputs.str()+"_robcogen_"+algorithm_name+".csv";
   output_csv.open (output_filename.c_str());

   SMOOTH(NBT)
   {
      if (_smooth != 0) output_csv << "\n";

      timer.tic();
      // Dynamics Algorithms
      #ifdef RNEA_ALG
         #if FLOATING
            invdyn.id(jForces, vdots[_smooth], g, vs[_smooth], qs[_smooth], qdots[_smooth], qddots[_smooth]);
         #else
            invdyn.id(taus[_smooth], qs[_smooth], qdots[_smooth], qddots[_smooth]);
         #endif
      #elif  CRBA_ALG
         jsimdyn.update(qs[_smooth]);
      #elif  ABA_ALG
         #if FLOATING
            frwdyn.fd(qddots[_smooth], vdots[_smooth], vs[_smooth], g, qs[_smooth], qdots[_smooth], taus[_smooth]);
         #else
            frwdyn.fd(qddots[_smooth], qs[_smooth], qdots[_smooth], taus[_smooth]);
         #endif
      #else
         #if FLOATING
            invdyn.id(jForces, vdots[_smooth], g, vs[_smooth], qs[_smooth], qdots[_smooth], qddots[_smooth]);
         #else
            invdyn.id(taus[_smooth], qs[_smooth], qdots[_smooth], qddots[_smooth]);
         #endif
      #endif /* Dynamics Algorithms */
      time_data = timer.toc(TicToc::NS);

      output_csv << time_data;
   }

   output_csv.close();

   std::cout << "--" << std::endl;

  return 0;
}
