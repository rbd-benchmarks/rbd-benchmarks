#include <declarations.h>
#include <transforms.h>
#include <inertia_properties.h>
#include <inverse_dynamics.h>
#include <forward_dynamics.h>
#include <jsim.h>

#include <stdio.h>

#include "csv_reader.h"

#define VSIZE 6

#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth)

#define THRESHOLD (0.01)

using namespace iit::ROBOT_NAMESPACE;

int main(void) {

  // Set robot model
  std::string robot_model = ROBOT_MODEL;
  std::cout << "Robot Model = " << robot_model << std::endl;

  // Import CSV inputs
  std::string input_filename = RBD_BENCHMARKS_DIR"/csv/robcogen/"+robot_model+"_inputs.csv";
  std::ifstream input_csv(input_filename.c_str());
  CSVRow row;
  input_csv >> row;

  JointState  qs     = JointState::Zero();
  JointState  qdots  = JointState::Zero();
  JointState  taus   = JointState::Zero();
  JointState  qddots = JointState::Zero();

#if FLOATING
  iit::rbd::VelocityVector vs    = iit::rbd::VelocityVector::Zero();
  iit::rbd::VelocityVector vdots = iit::rbd::VelocityVector::Zero();
  iit::rbd::VelocityVector g = iit::rbd::VelocityVector::Zero();
  JointState jForces = JointState::Zero();
#endif

  int tot_q = JointSpaceDimension;

  std::cout << "qs, qdots = " << tot_q << ", " << tot_q << std::endl;

  int col, start_col;
  input_csv >> row;
  start_col = 0;                    // 0
  for(int j=0;j<tot_q;++j)
  {
    col = start_col+j;
    qs[j] = atof(row[col].c_str());
  }
  start_col += tot_q;                // 1xQ

#if FLOATING
  for(int j=0;j<VSIZE;++j)
  {
    col = start_col+j;
    vs[j] = atof(row[col].c_str());
  }
  start_col += VSIZE;        // 1xQ + 6
#endif

  for(int j=0;j<tot_q;++j)
  {
    col = start_col+j;
    qdots[j] = atof(row[col].c_str());
  }
  start_col += tot_q;      // 2xQ + 6

#if FLOATING    
  for(int j=0;j<VSIZE;++j)
  {
    col = start_col+j;
    vdots[j] = atof(row[col].c_str());
  }
  start_col += VSIZE;    // 2xQ + 2x6
#endif

  for(int j=0;j<tot_q;++j)
  {
    col = start_col+j;
    qddots[j] = atof(row[col].c_str());
  }

#if FLOATING
  start_col += VSIZE;
#endif

  start_col += tot_q;   // 3xQ + 3x6
  for(int j=0;j<tot_q;++j)
  {
    col = start_col+j;
    taus[j] = atof(row[col].c_str());
  }


  ForceTransforms ftransforms;
  MotionTransforms transforms;
  dyn::InertiaProperties inertias;
  dyn::InverseDynamics invdyn(inertias, transforms);
  dyn::ForwardDynamics frwdyn(inertias, transforms);
  dyn::JSIM jsimdyn(inertias, ftransforms);

  std::cout << "--" << std::endl;

  // Dynamics Algorithms
#ifdef RNEA_ALG
  std::cout << "RNEA" << std::endl;
#if FLOATING
  invdyn.id(jForces, vdots, g, vs, qs, qdots, qddots);
#else
  invdyn.id(taus, qs, qdots, qddots);
#endif

  std::string nmcsv = "_inverse_dynamics_expected.csv";
#elif CRBA_ALG
  std::cout << "CRBA" << std::endl;
  jsimdyn.update(qs);
  std::string nmcsv = "_mass_matrix_expected.csv";
#elif ABA_ALG
  std::cout << "ABA" << std::endl;
#if FLOATING
  frwdyn.fd(qddots, vdots, vs, g, qs, qdots, taus);
#else
  frwdyn.fd(qddots, qs, qdots, taus);
#endif

  std::string nmcsv = "_dynamics_expected.csv";
#endif /* Dynamics Algorithms */
  std::cout << "--" << std::endl;

  std::cout << "TEST RESULTS" << std::endl;

  // Import CSV output
  std::string output_filename = RBD_BENCHMARKS_DIR"/csv/robcogen/"+robot_model+nmcsv;
  std::ifstream output_csv(output_filename.c_str());
  output_csv >> row;

  bool flag_wrong = false;
#ifdef RNEA_ALG
#if FLOATING
  for(int j=0;j<tot_q;++j)
  {
    double result = jForces[j];
    double truth  = atof(row[j + VSIZE].c_str());
#else
  for(int j=0;j<tot_q;++j)
  {
    double result = taus[j];
    double truth  = atof(row[j].c_str());
#endif

#elif  CRBA_ALG
  for(int j=0;j<jsimdyn.rows() * jsimdyn.cols();++j)
  {
    double result = jsimdyn(j);
    double truth  = atof(row[j].c_str());
#elif  ABA_ALG
  int qdd_start = 0;
#if FLOATING
  printf("vdots :\n");
  for(int i=0; i < VSIZE; i++) {
    double result = vdots[i];
    double truth = atof(row[i].c_str());
    printf("%f == %f\n", result, truth);
    printf("%f\n", result);
    double delta = fabs(truth - result);
    //printf("%f\n", delta);
    if(delta > THRESHOLD)
      flag_wrong=true;
  }
  qdd_start += 6;
#endif
 for(int j=0;j<tot_q;++j)
  {
    double result = qddots[j];
    double truth  = atof(row[qdd_start + j].c_str());
#endif 

    printf("%f == %f\n", result, truth);
    //printf("%f\n", result);
    double delta = fabs(truth - result);
    printf("%f\n", delta);
    if(delta > THRESHOLD)
      flag_wrong=true;
  }
  
  if(flag_wrong){
    std::cout << "Wrong results!!!\n" << std::endl;
    return 1;
  }
  else
    std::cout << "All the results were good" << std::endl;

  return 0;
}
