#ifdef __cplusplus
extern "C" {
#endif

  // Standard headers
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

  // Argument parsing
#include <ctype.h>

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

#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth)

#define TRESHOLD (0.00001)

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
  std::string robot_model=ROBOT_MODEL;

  // Parse arguments.
  char *urdf = RBD_BENCHMARKS_DIR"/description/urdf/"ROBOT_MODEL".urdf";
  bool floating = FLOATING;
  char *csv = RBD_BENCHMARKS_DIR"/csv/rbdjl/"ROBOT_MODEL"_inputs.csv";
  int c;
  opterr = 0;


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
  input_csv >> row;
  int tot_q, tot_qdot; // FIXME: What vector sizes do I expect from each model?
  tot_q    = nq;
  tot_qdot = nv;
  std::cout << "qs, qdots = " << tot_q << ", " << tot_qdot << std::endl;
  int col, start_col;
  input_csv >> row;
  start_col = 0;                 // 0
  for(int j=0;j<tot_q;++j)
  {
    col = start_col+j;
    q_data[j] = atof(row[col].c_str());
  }
  start_col = tot_q;               // 1xQ
  for(int j=0;j<tot_qdot;++j)
  {
    col = start_col+j;
    v_data[j] = atof(row[col].c_str());
  }
  start_col = tot_q+tot_qdot;      // 1xQ+1xQd
  for(int j=0;j<tot_qdot;++j)
  {
    col = start_col+j;
    vd_desired_data[j] = atof(row[col].c_str());
  }
  start_col = tot_q+2*tot_qdot;  // 1xQ+2xQd
  for(int j=0;j<tot_qdot;++j)
  {
    col = start_col+j;
    tau_data[j] = atof(row[col].c_str());
  }

  std::cout << "--" << std::endl;

  // Dynamics Algorithms
#ifdef RNEA_ALG
  std::cout << "RNEA" << std::endl;
  inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired);
  std::string nmcsv = "_inverse_dynamics_expected.csv";
#elif  CRBA_ALG
  std::cout << "CRBA" << std::endl;
  mass_matrix(M, state);
  std::string nmcsv = "_mass_matrix_expected.csv";
#elif  ABA_ALG
  std::cout << "ABA" << std::endl;
  dynamics(result, state, tau);
  std::string nmcsv = "_dynamics_expected.csv";
#endif /* Dynamics Algorithms */
  std::cout << "--" << std::endl;

  std::cout << "TEST RESULTS" << std::endl;
  // Import CSV output
  std::string output_filename = RBD_BENCHMARKS_DIR"/csv/rbdl/"+robot_model+nmcsv;
  std::ifstream output_csv(output_filename.c_str());
  output_csv >> row;

  bool flag_wrong = false;

#ifdef RNEA_ALG
  for(int j=0;j<tot_q;++j)
  {
    double res = tau_data[j];
    double truth = atof(row[j].c_str());
#elif  CRBA_ALG
  for(int colN = 0; colN < tot_qdot; colN++)
    for(int rowN = col; rowN < tot_qdot; rowN++)
    {
      int j = (colN * tot_qdot) + rowN;
      double res = M_data[j];
      double truth = atof(row[j].c_str());
#elif  ABA_ALG
  for(int j=0;j<tot_q;++j)
  {
    double res = vd_data[j];
    double truth = atof(row[j].c_str());
#endif 

    printf("%f == %f\n", res, truth);
    double delta = fabs(truth - res);
    printf("%f\n", delta);
    if(delta > TRESHOLD)
      flag_wrong=true;
  }

  if(flag_wrong){
      std::cout << "Wrong result !!!\n" << std::endl;
      return 1;
    }

  std::cout << "All the results were good" << std::endl;

  // Pop GC roots.
  JL_GC_POP();

  // Clean up and gracefully exit.
  jl_atexit_hook(retcode);
  return 0;
}
