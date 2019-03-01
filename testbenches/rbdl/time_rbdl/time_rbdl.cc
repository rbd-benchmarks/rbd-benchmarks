//
// Timing RBDL
//
// Based on example.cc from RBDL, and rbdl-bench.cpp from Pinocchio-Benchmarks
//

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "csv_reader.h"
#include "tictoc_timer.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {

   // Check arguments
   if (argc != 2)
   {
      std::cerr << "You have to specify a robot model. Choices are: iiwa, hyq, atlas" << std::endl;
      return 1;
   }

   // Set robot model
   bool floating_base;
   std::string robot_model(argv[1]);
   if (robot_model == "iiwa")
   {
      std::cout << "Robot Model = " << robot_model << std::endl;
      floating_base = false;
   }
   else if ((robot_model == "hyq")|(robot_model == "atlas"))
   {
      std::cout << "Robot Model = " << robot_model << std::endl;
      floating_base = true;
   }
   else
   {
      std::cerr << "Invalid robot model: " << robot_model << "\nChoices are: iiwa, hyq, atlas" << std::endl;
      return 2;
   }

   // Set up timer
   TicToc timer(TicToc::NS);
   const int NBT = 1000*100;

   // Import URDF model
   Model* model = NULL;
   model = new Model();
   std::string urdf_filename;
   urdf_filename = RBD_BENCHMARKS_DIR"/description/urdf/"+robot_model+".urdf";
   RigidBodyDynamics::Addons::URDFReadFromFile(urdf_filename.c_str(), model, floating_base);
   model->gravity = Math::Vector3d(0,0,0);
   int dof = model->dof_count;
   std::cout << "dof = " << dof << std::endl;

   // Import CSV inputs
   std::string input_filename;
   input_filename = RBD_BENCHMARKS_DIR"/csv/rbdl/"+robot_model+"_inputs.csv";
   std::ifstream input_csv(input_filename.c_str());
   CSVRow row;
   std::vector<VectorNd> qs     (NBT, VectorNd::Zero(model->q_size));
   std::vector<VectorNd> qdots  (NBT, VectorNd::Zero(model->qdot_size));
   std::vector<VectorNd> taus   (NBT, VectorNd::Zero(model->qdot_size));
   std::vector<VectorNd> qddots (NBT, VectorNd::Zero(model->qdot_size));
   input_csv >> row;
   int tot_q, tot_qdot;
   tot_q    = model->q_size;
   tot_qdot = model->qdot_size;
   std::cout << "qs, qdots = " << tot_q << ", " << tot_qdot << std::endl;
   int col, start_col;
   for(int i=0;i<NBT;++i)
   {
      input_csv >> row;
      start_col = 0;                 // 0
      for(int j=0;j<tot_q;++j)
      {
            col = start_col+j;
            qs[i][j] = atof(row[col].c_str());
      }
      start_col = tot_q;               // 1xQ
      for(int j=0;j<tot_qdot;++j)
      {
            col = start_col+j;
            qdots[i][j] = atof(row[col].c_str());
      }
      start_col = tot_q+tot_qdot;      // 1xQ+1xQd
      for(int j=0;j<tot_qdot;++j)
      {
            col = start_col+j;
            qddots[i][j] = atof(row[col].c_str());
      }
      start_col = tot_q+2*tot_qdot;  // 1xQ+2xQd
      for(int j=0;j<tot_qdot;++j)
      {
            col = start_col+j;
            taus[i][j] = atof(row[col].c_str());
      }
   }

   // Initialize some output variables
   double time_rnea;
   double time_crba;
   double time__aba;

   std::cout << "--" << std::endl;

   // RNEA
   timer.tic();
   SMOOTH(NBT)
   {
      InverseDynamics(*model,qs[_smooth],qdots[_smooth],qddots[_smooth],taus[_smooth]);
   }
   time_rnea = timer.toc(TicToc::NS)/NBT;
   std::cout << "RNEA = \t\t" << time_rnea << " " << timer.unitName(TicToc::NS) << std::endl;

   // CRBA
   MatrixNd H = MatrixNd::Zero ((size_t) model->dof_count, (size_t) model->dof_count);
   timer.tic();
   SMOOTH(NBT)
   {
      CompositeRigidBodyAlgorithm(*model,qs[_smooth],H);
   }
   time_crba = timer.toc(TicToc::NS)/NBT;
   std::cout << "CRBA = \t\t" << time_crba << " " << timer.unitName(TicToc::NS) << std::endl;

   // ABA
   timer.tic();
   SMOOTH(NBT)
   {
      ForwardDynamics(*model,qs[_smooth],qdots[_smooth],taus[_smooth],qddots[_smooth]);
   }
   time__aba = timer.toc(TicToc::NS)/NBT;
   std::cout << "ABA  = \t\t" << time__aba << " " << timer.unitName(TicToc::NS) << std::endl;

   std::cout << "--" << std::endl;

   // Write CSV outputs
   std::ofstream output_csv;
   std::string output_filename;
   std::stringstream num_inputs;
   num_inputs << NBT;
   output_filename = RESULTS_DIR"/time_"+robot_model+"_"+num_inputs.str()+"_rbdl.csv";
   output_csv.open (output_filename.c_str());
   output_csv << time_rnea << ",";
   output_csv << time_crba << ",";
   output_csv << time__aba;
   output_csv.close();

   delete model;
   return 0;
}
