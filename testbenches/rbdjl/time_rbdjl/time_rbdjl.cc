#ifdef __cplusplus
extern "C" {
#endif

// Standard headers
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Argument parsing
#include <ctype.h>
#include <stdio.h>
//#include <stdlib.h>
#include <unistd.h>

// Julia headers (for initialization and gc commands)
#include "uv.h"
#include "julia.h"

#ifdef __cplusplus
}
#endif

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "csv_reader.h"
#include "tictoc_timer.h"

#ifdef JULIA_DEFINE_FAST_TLS // only available in Julia v0.7 and above
JULIA_DEFINE_FAST_TLS()
#endif

// Declare C prototype of Julia functions
#ifdef __cplusplus
extern "C" {
#endif

jl_value_t* create_mechanism(char*, bool, int);
jl_value_t* create_state(jl_value_t*);
jl_value_t* create_dynamics_result(jl_value_t*);

void inverse_dynamics(jl_value_t*, jl_value_t*, jl_value_t*, jl_value_t*, jl_value_t*);
void mass_matrix(jl_value_t*, jl_value_t*);
void dynamics(jl_value_t*, jl_value_t*, jl_value_t*);

#ifdef __cplusplus
}
#endif

// modified from https://github.com/JuliaLang/julia/blob/8d6c1cebfd5cd97ebe33c9b84aa0f82335ed41a9/src/julia.h#L689
//#ifndef JL_GC_PUSH7
//#define JL_GC_PUSH7(arg1, arg2, arg3, arg4, arg5, arg6, arg7)                             \
//  void *__gc_stkf[] = {(void*)13, jl_pgcstack, arg1, arg2, arg3, arg4, arg5, arg6, arg7}; \
//  jl_pgcstack = (jl_gcframe_t*)__gc_stkf;
//#endif

int main(int argc, char *argv[])
{
   // Set up timer
   TicToc timer(TicToc::NS);
   const int NBT = 1000*100;

   // Parse arguments.
   char *urdf = NULL;
   bool floating = false;
   char *csv = NULL;
   int c;
   opterr = 0;

   while ((c = getopt(argc, argv, "u:fc:")) != -1)
      switch (c)
      {
      case 'u':
         urdf = optarg;
         break;
      case 'f':
         floating = true;
         break;
      case 'c':
         csv = optarg;
         break;
      default:
         abort();
      }
   if (!urdf) {
      fprintf (stderr, "Must pass in URDF argument (-u).");
      abort();
   }
   if (!csv) {
      fprintf (stderr, "Must pass in CSV argument (-c).");
      abort();
   }

   int retcode;
   int i;
   uv_setup_args(argc, argv); // no-op on Windows

   // initialization
   libsupport_init();

   // FIXME: jl_options.compile_enabled = JL_OPTIONS_COMPILE_OFF;
   // JULIAC_PROGRAM_LIBNAME defined on command-line for compilation
   jl_options.image_file = JULIAC_PROGRAM_LIBNAME;
   julia_init(JL_IMAGE_JULIA_HOME);

   // Make BLAS/LAPACK single threaded.
   jl_eval_string("using RigidBodyDynamics.LinearAlgebra"); // just LinearAlgebra doesn't work for some reason.
   jl_eval_string("BLAS.set_num_threads(1)");

   // GC roots.
   jl_value_t *mechanism = NULL, *state = NULL, *result = NULL;
   jl_value_t *vd_desired = NULL, *tau = NULL;
   JL_GC_PUSH5(&mechanism, &state, &result, &vd_desired, &tau);

   // Set number of BLAS threads to 1
   //jl_eval_string("using LinearAlgebra; BLAS.set_num_threads(1)");

   // Parse URDF.
   mechanism = create_mechanism(urdf, floating, 1);

   // Create MechanismState and DynamicsResult.
   state = create_state(mechanism);
   result = create_dynamics_result(mechanism);

   // Get state dimensions (possibly useful later).
   jl_eval_string("using RigidBodyDynamics");
   int nq = jl_unbox_int64(jl_call1(jl_get_function(jl_main_module, "num_positions"), state));
   int nv = jl_unbox_int64(jl_call1(jl_get_function(jl_main_module, "num_velocities"), state));

   // Get `jointwrenches` and `accelerations` work buffers/secondary outputs.
   // from result (for inverse dynamics)
   jl_value_t *jointwrenches = jl_get_field(result, "jointwrenches");
   jl_value_t *accelerations = jl_get_field(result, "accelerations");

   // Get `massmatrix` work buffer/output (for mass_matrix).
   jl_value_t *M = jl_get_field(result, "massmatrix");

   // Get `vd` (joint accelerations) work buffer/output (for dynamics)
   jl_value_t *vd = jl_get_field(result, "v̇");

   // Get/create input data arrays
   jl_function_t *configuration = jl_get_function(jl_main_module, "configuration");
   jl_function_t *velocity = jl_get_function(jl_main_module, "velocity");
   jl_function_t *similar = jl_get_function(jl_main_module, "similar");
   jl_value_t* q = jl_call1(configuration, state);
   jl_value_t* v = jl_call1(velocity, state);
   vd_desired = jl_call1(similar, v); // for inverse dynamics
   tau = jl_call1(similar, v);

   // Raw arrays for setting inputs and retrieving results.
   // Note that `q`, `v`, etc. are `RigidBodyDynamics.CustomCollections.SegmentedVectors`,
   // so we have to get their parents (regular `Vector`s) first.
   // Similarly, `M` is a `Symmetric`, so get its backing array using `parent` as well.
   jl_function_t *parent = jl_get_function(jl_main_module, "parent");
   double *q_data = (double*)jl_array_data(jl_call1(parent, q));
   double *v_data = (double*)jl_array_data(jl_call1(parent, v));
   double *vd_desired_data = (double*)jl_array_data(jl_call1(parent, vd_desired));
   double *tau_data = (double*)jl_array_data(jl_call1(parent, tau));
   double *vd_data = (double*)jl_array_data(jl_call1(parent, vd));
   double *M_data = (double*)jl_array_data(jl_call1(parent, M));

   // Import CSV inputs
   std::string input_filename;
   input_filename = csv;
   std::ifstream input_csv(input_filename.c_str());
   CSVRow row;
   std::vector<std::vector<double> > qs     (NBT, std::vector<double>(nq));
   std::vector<std::vector<double> > qdots  (NBT, std::vector<double>(nv));
   std::vector<std::vector<double> > qddots (NBT, std::vector<double>(nv));
   std::vector<std::vector<double> > taus   (NBT, std::vector<double>(nv));
   input_csv >> row;
   int tot_q, tot_qdot; // FIXME: What vector sizes do I expect from each model?
   tot_q    = nq;
   tot_qdot = nv;
   std::cout << "qs, qdots = " << tot_q << ", " << tot_qdot << std::endl;
   int col, start_col;
   for(int k=0;k<NBT;++k)
   {
      input_csv >> row;
      start_col = 0;                 // 0
      for(int j=0;j<tot_q;++j)
      {
         col = start_col+j;
         qs[k][j] = atof(row[col].c_str());
      }
      start_col = tot_q;               // 1xQ
      for(int j=0;j<tot_qdot;++j)
      {
            col = start_col+j;
            qdots[k][j] = atof(row[col].c_str());
      }
      start_col = tot_q+tot_qdot;      // 1xQ+1xQd
      for(int j=0;j<tot_qdot;++j)
      {
            col = start_col+j;
            qddots[k][j] = atof(row[col].c_str());
      }
      start_col = tot_q+2*tot_qdot;  // 1xQ+2xQd
      for(int j=0;j<tot_qdot;++j)
      {
            col = start_col+j;
            taus[k][j] = atof(row[col].c_str());
      }
   }

   // Initialize some output variables
   double time_rnea;
   double time_crba;
   double time__aba;

   std::cout << "--" << std::endl;

   // RNEA
   // Precompile. FIXME: Run before measurement, to precompile Julia functions...
   for (i=0; i< nq; i++) {
      q_data[i] = qs[NBT-1][i];
   }
   for (i=0; i< nv; i++) {
      v_data[i] = qdots[NBT-1][i];
      vd_desired_data[i] = qddots[NBT-1][i];
      tau_data[i] = taus[NBT-1][i];
   }
   inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired);
   // Start measurement
   timer.tic();
   SMOOTH(NBT)
   {
      // Copy inputs. TODO: Don't want to do this between measurements...
      for (i=0; i< nq; i++) {
         q_data[i] = qs[_smooth][i];
      }
      for (i=0; i< nv; i++) {
         v_data[i] = qdots[_smooth][i];
         vd_desired_data[i] = qddots[_smooth][i];
         tau_data[i] = taus[_smooth][i];
      }
      inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired);
   }
   time_rnea = timer.toc(TicToc::NS)/NBT;
   std::cout << "RNEA = \t\t" << time_rnea << " " << timer.unitName(TicToc::NS) << std::endl;

   // CRBA
   // Precompile. FIXME: Run before measurement, to precompile Julia functions...
   for (i=0; i< nq; i++) {
      q_data[i] = qs[NBT-1][i];
   }
   for (i=0; i< nv; i++) {
      v_data[i] = qdots[NBT-1][i];
      vd_desired_data[i] = qddots[NBT-1][i];
      tau_data[i] = taus[NBT-1][i];
   }
   mass_matrix(M, state);
   // Start measurement
   timer.tic();
   SMOOTH(NBT)
   {
      // Copy inputs. TODO: Don't want to do this between measurements...
      for (i=0; i< nq; i++) {
         q_data[i] = qs[_smooth][i];
      }
      for (i=0; i< nv; i++) {
         v_data[i] = qdots[_smooth][i];
         vd_desired_data[i] = qddots[_smooth][i];
         tau_data[i] = taus[_smooth][i];
      }
      mass_matrix(M, state);
   }
   time_crba = timer.toc(TicToc::NS)/NBT;
   std::cout << "CRBA = \t\t" << time_crba << " " << timer.unitName(TicToc::NS) << std::endl;

   // ABA
   // Precompile. FIXME: Run before measurement, to precompile Julia functions...
   for (i=0; i< nq; i++) {
      q_data[i] = qs[NBT-1][i];
   }
   for (i=0; i< nv; i++) {
      v_data[i] = qdots[NBT-1][i];
      vd_desired_data[i] = qddots[NBT-1][i];
      tau_data[i] = taus[NBT-1][i];
   }
   dynamics(result, state, tau);
   // Start measurement
   timer.tic();
   SMOOTH(NBT)
   {
      // Copy inputs. TODO: Don't want to do this between measurements...
      for (i=0; i< nq; i++) {
         q_data[i] = qs[_smooth][i];
      }
      for (i=0; i< nv; i++) {
         v_data[i] = qdots[_smooth][i];
         vd_desired_data[i] = qddots[_smooth][i];
         tau_data[i] = taus[_smooth][i];
      }
      dynamics(result, state, tau);
   }
   time__aba = timer.toc(TicToc::NS)/NBT;
   std::cout << "ABA  = \t\t" << time__aba << " " << timer.unitName(TicToc::NS) << std::endl;

   std::cout << "--" << std::endl;

   // Identify Robot Model
   std::string urdf_filename;
   std::string robot_model;
   std::string name_iiwa  = "iiwa";
   std::string name_hyq   = "hyq";
   std::string name_atlas = "atlas";
   urdf_filename = urdf;
   std::size_t found_iiwa  = urdf_filename.find(name_iiwa);
   std::size_t found_hyq   = urdf_filename.find(name_hyq);
   std::size_t found_atlas = urdf_filename.find(name_atlas);
   if      (found_iiwa!=std::string::npos)
      robot_model = name_iiwa;
   else if (found_hyq!=std::string::npos)
      robot_model = name_hyq;
   else if (found_atlas!=std::string::npos)
      robot_model = name_atlas;
   else {
      std::cerr << "Invalid robot model: " << robot_model << "\nChoices are: iiwa, hyq, atlas" << std::endl;
      return 1;
   }

   // Write CSV outputs
   std::ofstream output_csv;
   std::string output_filename;
   std::stringstream num_inputs;
   num_inputs << NBT;
   output_filename = RESULTS_DIR"/time_"+robot_model+"_"+num_inputs.str()+"_rbdjl.csv";
   output_csv.open (output_filename.c_str());
   output_csv << time_rnea << ",";
   output_csv << time_crba << ",";
   output_csv << time__aba;
   output_csv.close();

   // Pop GC roots.
   JL_GC_POP();

   // Clean up and gracefully exit.
   jl_atexit_hook(retcode);
   return 0;
}
