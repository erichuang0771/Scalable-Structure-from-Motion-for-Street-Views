// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.
#include "BA.h"

// Read a Bundle Adjustment in the Large dataset.

using namespace std;

int local_bundle_adjustment(arma::fmat K,
                            arma::fmat& poseA,
                            arma::fmat& poseB,
                            arma::fmat& point2DA,
                            arma::fmat& point2DB,
                            arma::fmat& point4D){
  cout<<"BA connected!....\n";
  cout<<"pose: "<<poseA<<"\n pose: "<<poseB<<endl;


  cout<<"*****BA:"<<point2DA.n_rows<<" B:"<<point2DB.n_rows<<endl;;
  ceres::Problem problem;
  // double* observation_A = new double[point2DA.n_rows*2];
  // double* observation_B = new double[point2DA.n_rows*2];
  arma::fmat R = poseA.submat(0,0,2,2);
  float* angles = new float[3];

  ceres::RotationMatrixToAngleAxis(R.memptr(),angles);


  double* camParameters_A = new double[9];
  camParameters_A[0] = angles[0];
  camParameters_A[1] = angles[1];
  camParameters_A[2] = angles[2];
  camParameters_A[3] = poseA.at(0,3);
  camParameters_A[4] = poseA.at(1,3);
  camParameters_A[5] = poseA.at(2,3);
  // camParameters_A[6] = (K.at(0,0)+K.at(1,1))/2;
  // camParameters_A[7] = K(0,2);
  // camParameters_A[8] = K(1,2);

  cout<<"point4D.n_rows: "<<point4D.n_rows<<endl;
  //set up camA
  arma::fmat R2 = poseB.submat(0,0,2,2);
  float* angles2 = new float[3];
  ceres::RotationMatrixToAngleAxis(R2.memptr(),angles2);
 cout<<"angle rot: "<<R2<<endl;
  cout<<"angle rot: "<<angles2[0]<<" | "<<angles2[1]<<" | "<<angles2[2]<<endl;
  //return 0;


  double* camParameters_B = new double[9];
  //set up camB
  camParameters_B[0] = angles2[0];
  camParameters_B[1] = angles2[1];
  camParameters_B[2] = angles2[2];
  camParameters_B[3] = poseB.at(0,3);
  camParameters_B[4] = poseB.at(1,3);
  camParameters_B[5] = poseB.at(2,3);
  // camParameters_B[6] = (K.at(0,0)+K.at(1,1))/2;
  // camParameters_B[7] = K(0,2);
  // camParameters_B[8] = K(1,2);
  cout<<"K(0,2)"<<K(0,2)<<"  |  "<<"K(1,2)"<<K(1,2)<<endl;
  cout<<"******BA: point4D.size(): "<<point4D.n_rows<<endl;

  double* point4D_BA = new double[point4D.n_rows*3];


  for (int i = 0; i <  point2DA.n_rows; ++i) {
    //set up obesrvation A
    //set up 3D point
    point4D_BA[i*3+0] = point4D(i,0);
    point4D_BA[i*3+1] = point4D(i,1);
    point4D_BA[i*3+2] = point4D(i,2);

   // cout<<"obs: "<<observation_A[i*2]<<"   |   "<<observation_A[i*2+1]<<endl;
    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(point2DA(i,0),
                                          point2DA(i,1));

    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss */,
                             camParameters_A,
                             point4D_BA+i*3);
  }
  std::cout<<"set up camera A done!!!!!!"<<std::endl;
  for (int i = 0; i <  point2DA.n_rows; ++i) {
    //set up obesrvation B

    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(point2DB(i,0),
                                         point2DB(i,1));

    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss */,
                             camParameters_B,
                             point4D_BA+i*3);
  }
  std::cout<<"set up camera B done!!!!!!"<<std::endl;


  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = true ;
            options.max_num_iterations = 50;
            //  options.function_tolerance = 1e-3;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
            double* RR = new double[9];
            ceres::AngleAxisToRotationMatrix<double>(camParameters_A,RR);
            arma::mat R_1(reinterpret_cast<double*>(RR),3,3);
            arma::mat T_1(reinterpret_cast<double*>(&camParameters_A[3]),3,1);
            arma::mat T_2(reinterpret_cast<double*>(&camParameters_B[3]),3,1);
            R_1 = arma::join_rows(R_1,T_1);
            R_1 = K*R_1;
            R_1.save("BA_camProj_1.mat",arma::raw_ascii);

            double* RR2 = new double[9];
            ceres::AngleAxisToRotationMatrix<double>(camParameters_B,RR2);
            arma::mat R_2(reinterpret_cast<double*>(RR2),3,3);
            R_2 = arma::join_rows(R_2,T_2);
            R_2 = K*R_2;
            R_2.save("BA_camProj_2.mat",arma::raw_ascii);

for(unsigned i = 0; i < 6; ++i) {
  /* code */
  cout<<" camA: "<<camParameters_A[i];
 // cout<<"camB: "<<camParameters_B[i]<<endl;
}
// ceres::AngleAxisToRotationMatrix<double>(camParameters_B,RR);

cout<<"\n\n";
for(unsigned i = 0; i < 6; ++i) {
  /* code */
  cout<<" camB: "<<camParameters_B[i];
 // cout<<"camB: "<<camParameters_B[i]<<endl;
}
cout<<"\n\n";
// cout<<"K: "<<K(0,2)<<" | "<<K(1,2)<<endl;
for (size_t i = 0; i < point4D.n_rows; i++) {
  /* code */
  point4D(i,0) = point4D_BA[i*3+0];
  point4D(i,1) = point4D_BA[i*3+1];
  point4D(i,2) = point4D_BA[i*3+2];
}
//save BA pose:

static int ba_cnt = 0;
point4D.save("BA_"+to_string(ba_cnt)+".mat",arma::raw_ascii);ba_cnt++;
std::cout << "\n saved BA results \n" << std::endl;
return 0;
}








class BALProblem {
 public:
  ~BALProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observations()       const { return num_observations_;               }
  const double* observations() const { return observations_;                   }
  double* mutable_cameras()          { return parameters_;                     }
  double* mutable_points()           { return parameters_  + 9 * num_cameras_; }
  double* mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 9;
  }

  double* mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
  }

  bool LoadFile(const char* filename) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
      return false;
    };
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);
    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];
    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];
    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
      }
    }
    for (int i = 0; i < num_parameters_; ++i) {
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    return true;
  }
 private:
  template<typename T>
  void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }
  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;
  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
};
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).

int main_(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  if (argc != 2) {
    std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
    return 1;
  }
  BALProblem bal_problem;
  if (!bal_problem.LoadFile(argv[1])) {
    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
    return 1;
  }
  const double* observations = bal_problem.observations();
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0],
                                         observations[2 * i + 1]);
    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss */,
                             bal_problem.mutable_camera_for_observation(i),
                             bal_problem.mutable_point_for_observation(i));
  }
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport() << "\n";
  return 0;
}
