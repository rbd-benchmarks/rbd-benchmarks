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
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "csv_reader.h"

#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth)

#define ERROR_RETURN(retval) { fprintf(stderr, "Error %d %s:line %d: \n", retval,__FILE__,__LINE__);  exit(retval); }

#define TRESHOLD (0.00001)

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

int main() {

  using namespace Eigen;
  using namespace pinocchio;
  
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
  Model model;
  std::string urdf_filename;
  urdf_filename = RBD_BENCHMARKS_DIR"/description/urdf/"+robot_model+".urdf";
  if (floating_base)
    pinocchio::urdf::buildModel(urdf_filename,JointModelFreeFlyer(),model);
  else
    pinocchio::urdf::buildModel(urdf_filename,model);
  
  model.gravity = Eigen::Vector3d(0, 0, 0);
  Data data(model);
  int dof = model.nv;
  std::cout << "dof = " << dof << std::endl;

  // Import CSV inputs
  std::string input_filename;
  input_filename = RBD_BENCHMARKS_DIR"/csv/pinocchio/"+robot_model+"_inputs.csv";
  std::ifstream input_csv(input_filename.c_str());

  CSVRow row;
  VectorXd qs     = VectorXd::Zero(model.nq);
  VectorXd qdots  = VectorXd::Zero(model.nv);
  VectorXd qddots = VectorXd::Zero(model.nv);
  VectorXd taus   = VectorXd::Zero(model.nv);

  input_csv >> row;
  int tot_q, tot_qdot; // FIXME: What vector sizes do I expect from each model?
  tot_q    = model.nq;
  tot_qdot = model.nv;
  std::cout << "qs, qdots = " << tot_q << ", " << tot_qdot << std::endl;
  int col, start_col;
  input_csv >> row;
  start_col = 0.0;                 // 0
  for(int j=0;j<tot_q;++j)
  {
    col = start_col+j;
    qs[j] = atof(row[col].c_str());
  }
  //qs.segment<4>(3) /= qs.segment<4>(3).norm();
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
  start_col = tot_q+2*tot_qdot;  // 1xQ+2xQd
  for(int j=0;j<tot_qdot;++j)
  {
    col = start_col+j;
    taus[j] = atof(row[col].c_str());
  }


  std::cout << "--" << std::endl;

  // Dynamics Algorithms
#ifdef RNEA_ALG
  std::cout << "RNEA" << std::endl;
  const VectorXd& returned_value = rnea(model,data,qs,qdots,qddots);
  std::string nmcsv = "_inverse_dynamics_expected.csv";
#elif  CRBA_ALG
  std::cout << "CRBA" << std::endl;
  const MatrixXd& returned_value = crba(model,data,qs);
  std::string nmcsv = "_mass_matrix_expected.csv";
#elif  ABA_ALG
  std::cout << "ABA" << std::endl;
  const VectorXd& returned_value = aba(model,data,qs,qdots, taus);
  std::string nmcsv = "_dynamics_expected.csv";
#endif /* Dynamics Algorithms */
  std::cout << "--" << std::endl;

  std::cout << "TEST RESULTS" << std::endl;

  // Import CSV output
  std::string output_filename = RBD_BENCHMARKS_DIR"/csv/pinocchio/"+robot_model+nmcsv;
  std::ifstream output_csv(output_filename.c_str());
  output_csv >> row;

  bool flag_wrong = false;
#ifdef RNEA_ALG
  for(int j=0;j<tot_qdot;++j)
  {
    double result = returned_value[j];
    double truth  = atof(row[j].c_str());
#elif  CRBA_ALG
  for(int j=0;j<tot_qdot * tot_qdot;++j)
  {
    if((j/tot_qdot) < (j%tot_qdot)) //Only the uper triangle of the matrix is computed
      continue;

    double result = returned_value(j);
    double truth  = atof(row[j].c_str());
#elif  ABA_ALG
  for(int j=0;j<tot_qdot;++j)
  {
    double result = returned_value[j];
    double truth  = atof(row[j].c_str());
#endif 

    printf("%f == %f\n", result, truth);
    double delta = fabs(truth - result);
    printf("%f\n", delta);
    if(delta > TRESHOLD)
      flag_wrong=true;
  }
  if(flag_wrong){
    std::cout << "Wrong results!!!" << std::endl;
    return 1;
  }
  else
    std::cout << "All the results were good" << std::endl;


  return 0;
}
