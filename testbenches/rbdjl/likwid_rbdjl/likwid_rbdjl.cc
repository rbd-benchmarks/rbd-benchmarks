//
// Benchmarking RBDJL using LIKWID
//
// Based on C-likwidAPI.c from LIKWID
//

#ifdef __cplusplus
extern "C" {
#endif

// Standard headers
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Argument parsing
#include <ctype.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>

// Julia headers (for initialization and gc commands)
#include "uv.h"
#include "julia.h"

#ifdef __cplusplus
}
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "csv_reader.h"
#include <likwid.h>

#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth)

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

   // LIKWID variables
   int j;
   int err;
   int* cpus;
   int gid;
   double likwid_result = 0.0;
   setenv("LIKWID_FORCE", "1", 1);

   // LIKWID Events
   std::string events_name;
   #ifdef BRANCH_EVENTS
   events_name = "branch";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,BR_MISP_RETIRED_ALL_BRANCHES:PMC0,BR_INST_RETIRED_ALL_BRANCHES:PMC1";
   #elif  L2CACHE_EVENTS
   events_name = "l2cache";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,L2_TRANS_ALL_REQUESTS:PMC0,L2_RQSTS_MISS:PMC1";
   #elif  L3CACHE_EVENTS
   events_name = "l3cache";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,MEM_LOAD_RETIRED_L3_HIT:PMC0,MEM_LOAD_RETIRED_L3_MISS:PMC1,UOPS_RETIRED_ALL:PMC2";
   #elif  ENERGY_EVENTS
   events_name = "energy";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,TEMP_CORE:TMP0,PWR_PKG_ENERGY:PWR0,PWR_PP0_ENERGY:PWR1,PWR_PP1_ENERGY:PWR2,PWR_DRAM_ENERGY:PWR3";
   #elif  MIX_EVENTS
   events_name = "mix";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,MEM_INST_RETIRED_ALL_LOADS:PMC0,MEM_INST_RETIRED_ALL_STORES:PMC1,MEM_INST_RETIRED_ALL:PMC2";
   #elif  STALL_EVENTS
   events_name = "stall";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,CYCLE_ACTIVITY_STALLS_TOTAL:PMC0,CYCLE_ACTIVITY_STALLS_MEM_ANY:PMC1,INT_MISC_RECOVERY_CYCLES_ANY:PMC2";
   #elif  MSTALL_EVENTS
   events_name = "mstall";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,CYCLE_ACTIVITY_STALLS_L2_PENDING:PMC0,CYCLE_ACTIVITY_STALLS_LDM_PENDING:PMC1,CYCLE_ACTIVITY_STALLS_L1D_PENDING:PMC2,CYCLE_ACTIVITY_CYCLES_NO_EXECUTE:PMC3";
   #elif  FLOPSDP_EVENTS
   events_name = "flopsdp";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,FP_ARITH_INST_RETIRED_128B_PACKED_DOUBLE:PMC0,FP_ARITH_INST_RETIRED_SCALAR_DOUBLE:PMC1,FP_ARITH_INST_RETIRED_256B_PACKED_DOUBLE:PMC2";
   #elif  FLOPSSP_EVENTS
   events_name = "flopssp";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,FP_ARITH_INST_RETIRED_128B_PACKED_SINGLE:PMC0,FP_ARITH_INST_RETIRED_SCALAR_SINGLE:PMC1,FP_ARITH_INST_RETIRED_256B_PACKED_SINGLE:PMC2";
   #elif  UOPS_EVENTS
   events_name = "uops";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2,UOPS_ISSUED_ANY:PMC0,UOPS_EXECUTED_THREAD:PMC1,UOPS_RETIRED_ALL:PMC2";
   #else
   events_name = "fixed";
   char estr[] = "INSTR_RETIRED_ANY:FIXC0,CPU_CLK_UNHALTED_CORE:FIXC1,CPU_CLK_UNHALTED_REF:FIXC2";
   #endif /* LIKWID Events */

   //perfmon_setVerbosity(3);

   // Load the LIKWID topology module and print some values.
   err = topology_init();
   if (err < 0) {
      printf("Failed to initialize LIKWID's topology module\n");
      return 1;
   }
   // CpuInfo_t contains global information like name, CPU family, ...
   CpuInfo_t info = get_cpuInfo();
   // CpuTopology_t contains information about the topology of the CPUs.
   CpuTopology_t topo = get_cpuTopology();
   // Create affinity domains. Commonly only needed when reading Uncore counters
   affinity_init();
   printf("Likwid example on a %s with %d CPUs\n", info->name, topo->numHWThreads);
   cpus = (int*)malloc(topo->numHWThreads * sizeof(int));
   if (!cpus)
      return 1;
   for (i=0;i<topo->numHWThreads;i++) {
      cpus[i] = topo->threadPool[i].apicId;
   }

   // Must be called before LIKWID perfmon_init() but only if you want to use another
   // access mode as the pre-configured one. For direct access (0) you have to
   // be root.
   //accessClient_setaccessmode(0);

   // Initialize the LIKWID perfmon module.
   err = perfmon_init(topo->numHWThreads, cpus);
   if (err < 0) {
     printf("Failed to initialize LIKWID's performance monitoring module\n");
     topology_finalize();
     return 1;
   }

   // Add LIKWID eventset string to the perfmon module.
   gid = perfmon_addEventSet(estr);
   if (gid < 0) {
     printf("Failed to add event string %s to LIKWID's performance monitoring module\n", estr);
     perfmon_finalize(); topology_finalize();
     return 1;
   }

   // Setup the LIKWID eventset identified by group ID (gid).
   err = perfmon_setupCounters(gid);
   if (err < 0) {
     printf("Failed to setup group %d in LIKWID's performance monitoring module\n", gid);
     perfmon_finalize(); topology_finalize();
     return 1;
   }

   // Count the LIKWID events in estr.
   int numEvents;
   char* ptr = strtok(estr,",");
   j = 0;
   while (ptr != NULL) {
      ptr = strtok(NULL,",");
      j++;
   }
   numEvents = j;

   // Set up number of iterations
   const int NBT = 1000*100;

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

   std::cout << "--" << std::endl;

   // Dynamics Algorithms
   std::string algorithm_name;
   #ifdef RNEA_ALG
   algorithm_name = "rnea";
   std::cout << "RNEA" << std::endl;
   #elif  CRBA_ALG
   algorithm_name = "crba";
   std::cout << "CRBA" << std::endl;
   #elif  ABA_ALG
   algorithm_name = "aba";
   std::cout << "ABA" << std::endl;
   #else
   algorithm_name = "rnea";
   std::cout << "RNEA" << std::endl;
   #endif /* Dynamics Algorithms */
   
   // Precompile. FIXME: Run before measurement, to precompile Julia functions...
   for (i=0; i< nq; i++) {
      q_data[i] = qs[NBT-1][i];
   }
   for (i=0; i< nv; i++) {
      v_data[i] = qdots[NBT-1][i];
      vd_desired_data[i] = qddots[NBT-1][i];
      tau_data[i] = taus[NBT-1][i];
   }
   // Dynamics Algorithms
   #ifdef RNEA_ALG
   inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired);
   #elif  CRBA_ALG
   mass_matrix(M, state);
   #elif  ABA_ALG
   dynamics(result, state, tau);
   #else
   inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired);
   #endif /* Dynamics Algorithms */

   // Start measurement
   // Start all LIKWID counters in the previously set up event set.
   err = perfmon_startCounters();
   if (err < 0) {
     printf("Failed to start counters for group %d for thread %d\n",gid, (-1*err)-1);
     perfmon_finalize(); topology_finalize();
     return 1;
   }
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
      // Dynamics Algorithms
      #ifdef RNEA_ALG
      inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired);
      #elif  CRBA_ALG
      mass_matrix(M, state);
      #elif  ABA_ALG
      dynamics(result, state, tau);
      #else
      inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired);
      #endif /* Dynamics Algorithms */
   }
   // Stop all LIKWID counters in the previously started event set.
   err = perfmon_stopCounters();
   if (err < 0) {
     printf("Failed to stop counters for group %d for thread %d\n",gid, (-1*err)-1);
     perfmon_finalize(); topology_finalize();
     return 1;
   }

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
   long long int csv_result;
   std::ofstream output_csv;
   std::string output_filename;
   std::stringstream num_inputs;
   num_inputs << NBT;
   output_filename = RESULTS_DIR"/"+events_name+"_"+robot_model+"_"+num_inputs.str()+"_rbdjl_"+algorithm_name+".csv";
   output_csv.open (output_filename.c_str());
   for (j = 0;j < numEvents; j++)
   {
      for (i = 0;i < topo->numHWThreads; i++)
      {
         likwid_result = perfmon_getResult(gid, j, i);
         csv_result = likwid_result;
         output_csv << csv_result << ",";
      }
      if (j < (numEvents-1))
         output_csv << "\n";
   }
   output_csv.close();

   // Done with LIKWID
   free(cpus);
   // Uninitialize the perfmon module.
   perfmon_finalize();
   affinity_finalize();
   // Uninitialize the topology module.
   topology_finalize();

   // Pop GC roots.
   JL_GC_POP();

   // Clean up and gracefully exit.
   jl_atexit_hook(retcode);
   return 0;
}
