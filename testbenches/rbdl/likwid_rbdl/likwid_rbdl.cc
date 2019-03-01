//
// Benchmarking RBDL using LIKWID
//
// Based on C-likwidAPI.c from LIKWID
//

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "csv_reader.h"
#include <likwid.h>

#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth)

#define NUM_EVENTS 2
#define ERROR_RETURN(retval) { fprintf(stderr, "Error %d %s:line %d: \n", retval,__FILE__,__LINE__);  exit(retval); }

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

   // LIKWID variables
   int i, j;
   int err;
   int* cpus;
   int gid;
   double result = 0.0;
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

   std::cout << "--" << std::endl;

   // Dynamics Algorithms
   std::string algorithm_name;
   #ifdef RNEA_ALG
   algorithm_name = "rnea";
   std::cout << "RNEA" << std::endl;
   #elif  CRBA_ALG
   algorithm_name = "crba";
   std::cout << "CRBA" << std::endl;
   MatrixNd H = MatrixNd::Zero ((size_t) model->dof_count, (size_t) model->dof_count);
   #elif  ABA_ALG
   algorithm_name = "aba";
   std::cout << "ABA" << std::endl;
   #else
   algorithm_name = "rnea";
   std::cout << "RNEA" << std::endl;
   #endif /* Dynamics Algorithms */
   // Start all LIKWID counters in the previously set up event set.
   err = perfmon_startCounters();
   if (err < 0) {
     printf("Failed to start counters for group %d for thread %d\n",gid, (-1*err)-1);
     perfmon_finalize(); topology_finalize();
     return 1;
   }
   SMOOTH(NBT)
   {
      // Dynamics Algorithms
      #ifdef RNEA_ALG
      InverseDynamics(*model,qs[_smooth],qdots[_smooth],qddots[_smooth],taus[_smooth]);
      #elif  CRBA_ALG
      CompositeRigidBodyAlgorithm(*model,qs[_smooth],H);
      #elif  ABA_ALG
      ForwardDynamics(*model,qs[_smooth],qdots[_smooth],taus[_smooth],qddots[_smooth]);
      #else
      InverseDynamics(*model,qs[_smooth],qdots[_smooth],qddots[_smooth],taus[_smooth]);
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

   // Write CSV outputs
   long long int csv_result;
   std::ofstream output_csv;
   std::string output_filename;
   std::stringstream num_inputs;
   num_inputs << NBT;
   output_filename = RESULTS_DIR"/"+events_name+"_"+robot_model+"_"+num_inputs.str()+"_rbdl_"+algorithm_name+".csv";
   output_csv.open (output_filename.c_str());
   for (j = 0;j < numEvents; j++)
   {
      for (i = 0;i < topo->numHWThreads; i++)
      {
         result = perfmon_getResult(gid, j, i);
         csv_result = result;
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

   delete model;
   return 0;
}
