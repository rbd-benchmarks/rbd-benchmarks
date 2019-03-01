//
// Timing Pinocchio
//
// Based on timings.cpp from Pinocchio
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "csv_reader.h"
#include "tictoc_timer.h"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

int main(int argc, const char ** argv)
{
   using namespace Eigen;
   using namespace pinocchio;

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

   // Setup timer
   TicToc timer(TicToc::NS);
   const int NBT = 1000*100;

   // Import URDF model
   Model model;
   std::string urdf_filename;
   urdf_filename = RBD_BENCHMARKS_DIR"/description/urdf/"+robot_model+".urdf";
   if (floating_base)
      pinocchio::urdf::buildModel(urdf_filename,JointModelFreeFlyer(),model);
   else
      pinocchio::urdf::buildModel(urdf_filename,model);
   model.gravity = Eigen::Vector3d(0,0,0);
   Data data(model);
   int dof = model.nv;
   std::cout << "dof = " << dof << std::endl;

   // Import CSV inputs
   std::string input_filename;
   input_filename = RBD_BENCHMARKS_DIR"/csv/pinocchio/"+robot_model+"_inputs.csv";
   std::ifstream input_csv(input_filename.c_str());
   CSVRow row;
   std::vector<VectorXd> qs     (NBT, VectorXd::Zero(model.nq));
   std::vector<VectorXd> qdots  (NBT, VectorXd::Zero(model.nv));
   std::vector<VectorXd> qddots (NBT, VectorXd::Zero(model.nv));
   std::vector<VectorXd> taus   (NBT, VectorXd::Zero(model.nv));
   input_csv >> row;
   int tot_q, tot_qdot;
   tot_q    = model.nq;
   tot_qdot = model.nv;
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
      rnea(model,data,qs[_smooth],qdots[_smooth],qddots[_smooth]);
   }
   time_rnea = timer.toc(TicToc::NS)/NBT;
   std::cout << "RNEA = \t\t" << time_rnea << " " << timer.unitName(TicToc::NS) << std::endl;

   // CRBA
   timer.tic();
   SMOOTH(NBT)
   {
      crba(model,data,qs[_smooth]);
   }
   time_crba = timer.toc(TicToc::NS)/NBT;
   std::cout << "CRBA = \t\t" << time_crba << " " << timer.unitName(TicToc::NS) << std::endl;

   // ABA
   timer.tic();
   SMOOTH(NBT)
   {
      aba(model,data,qs[_smooth],qdots[_smooth],taus[_smooth]);
   }
   time__aba = timer.toc(TicToc::NS)/NBT;
   std::cout << "ABA  = \t\t" << time__aba << " " << timer.unitName(TicToc::NS) << std::endl;

   std::cout << "--" << std::endl;

   // Write CSV outputs
   std::ofstream output_csv;
   std::string output_filename;
   std::stringstream num_inputs;
   num_inputs << NBT;
   output_filename = RESULTS_DIR"/time_"+robot_model+"_"+num_inputs.str()+"_pinocchio.csv";
   output_csv.open (output_filename.c_str());
   output_csv << time_rnea << ",";
   output_csv << time_crba << ",";
   output_csv << time__aba;
   output_csv.close();

   return 0;
}
