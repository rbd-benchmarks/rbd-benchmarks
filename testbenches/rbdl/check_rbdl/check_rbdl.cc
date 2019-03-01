#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "csv_reader.h"

#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth)

#define TRESHOLD (0.00001)

#define ERROR_RETURN(retval) { fprintf(stderr, "Error %d %s:line %d: \n", retval,__FILE__,__LINE__);  exit(retval); }

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {

  // Set robot model
  bool floating_base;
  std::string robot_model = ROBOT_MODEL;

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


  // Import URDF model
  Model* model = NULL;
  model = new Model();
  std::string urdf_filename;
  urdf_filename = RBD_BENCHMARKS_DIR"/description/urdf/"+robot_model+".urdf";
  RigidBodyDynamics::Addons::URDFReadFromFile(urdf_filename.c_str(), model, floating_base, true);
  model->gravity = Math::Vector3d(0, 0, 0);
  int dof = model->dof_count;
  std::cout << "dof = " << dof << std::endl;

  // Import CSV inputs
  std::string input_filename;
  input_filename = RBD_BENCHMARKS_DIR"/csv/rbdl/"+robot_model+"_inputs.csv";
  std::ifstream input_csv(input_filename.c_str());
  CSVRow row;
  VectorNd qs     =  VectorNd::Zero(model->q_size);
  VectorNd qdots  = VectorNd::Zero(model->qdot_size);
  VectorNd taus   = VectorNd::Zero(model->qdot_size);
  VectorNd qddots = VectorNd::Zero(model->qdot_size);
  input_csv >> row;
  int tot_q, tot_qdot; 
  tot_q    = model->q_size;
  tot_qdot = model->qdot_size;
  std::cout << "qs, qdots = " << tot_q << ", " << tot_qdot << std::endl;
  int col, start_col;
  input_csv >> row;
  start_col = 0.0;                 // 0
  for(int j=0;j<tot_q;++j)
  {
    col = start_col+j;
    qs[j] = atof(row[col].c_str());
  }
  start_col = tot_q;               // 1xQ
  for(int j=0;j<tot_qdot;++j)
  {
    col = start_col+j;
    qdots[j] = atof(row[col].c_str());
  }
  start_col = tot_q+tot_qdot;      // 1xQ+1xQd
  for(int j=0;j<tot_qdot;++j)
  {
    col = start_col+j;
    qddots[j] = atof(row[col].c_str());
  }
  start_col = tot_q+2.0*tot_qdot;  // 1xQ+2xQd
  for(int j=0;j<tot_qdot;++j)
  {
    col = start_col+j;
    taus[j] = atof(row[col].c_str());
  }

  std::cout << "--" << std::endl;

  // Dynamics Algorithms
#ifdef RNEA_ALG
  std::cout << "RNEA" << std::endl;
  InverseDynamics(*model,qs,qdots,qddots,taus);
  std::string nmcsv = "_inverse_dynamics_expected.csv";
#elif  CRBA_ALG
  std::cout << "CRBA" << std::endl;
  MatrixNd H = MatrixNd::Zero ((size_t) model->dof_count, (size_t) model->dof_count);
  CompositeRigidBodyAlgorithm(*model,qs,H);
  std::string nmcsv = "_mass_matrix_expected.csv";
#elif  ABA_ALG
  std::cout << "ABA" << std::endl;
  ForwardDynamics(*model,qs,qdots,taus,qddots);
  std::string nmcsv = "_dynamics_expected.csv";

#endif /* Dynamics Algorithms */

  std::cout << "--" << std::endl;

  std::cout << "TEST RESULTS" << std::endl;
  // Import CSV output
  std::string output_filename = RBD_BENCHMARKS_DIR"/csv/rbdl/"+robot_model+nmcsv;
  std::ifstream output_csv(output_filename.c_str());
  output_csv >> row;

  bool flag_wrong=false;
#ifdef RNEA_ALG
  for(int j=0;j<tot_qdot;++j)
  {
    double result = taus[j];
    double truth  = atof(row[j].c_str());
#elif  CRBA_ALG
  for(int j=0;j<tot_qdot * tot_qdot;++j)
  {
    double result = H(j);
    double truth  = atof(row[j].c_str());
#elif  ABA_ALG
  for(int j=0;j<tot_qdot;++j)
  {
    double result = qddots[j];
    double truth  = atof(row[j].c_str());
#endif 

    printf("%f == %f\n", result, truth);
    double delta = fabs(truth - result);
    printf("%f\n", delta);
    if(delta > TRESHOLD) {
      flag_wrong=true;
    }
  }

  if(flag_wrong){
    std::cout << "Wrong results!!!\n" << std::endl;
    return 1;
  }


  std::cout << "All the results were good" << std::endl;

  delete model;
  return 0;
}
