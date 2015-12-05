#ifndef BA
#define BA

#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/loss_function.h"
#include <armadillo>
#include "OpenSfM.h"

using namespace std;

struct SnavelyReprojectionError {
		SnavelyReprojectionError(double observed_x, double observed_y)
		: observed_x(observed_x), observed_y(observed_y) {}
		template <typename T>
		bool operator()(const T* const camera,
			const T* const point,
			T* residuals) const {
				// camera[0,1,2] are the angle-axis rotation.
				T p[3];
				ceres::AngleAxisRotatePoint(camera, point, p);
				//  T* R = new T[9];
				// ceres::AngleAxisToRotationMatrix(camera,R);
				//
				// for (size_t i = 0; i < 9; i++) {
				// 	/* code */
				// 	std::cout<<R[i]<<" |";
				// }
				p[0] += camera[3];
				p[1] += camera[4];
				p[2] += camera[5];
				// Compute the center of distortion. The sign change comes from
				// the camera model that Noah Snavely's Bundler assumes, whereby
				// the camera coordinate system has a negative z axis.
				T xp =  p[0] / p[2];
				T yp =  p[1] / p[2];
				// Compute final projected point position.
				const T& focal = T(1520.4);
				const T& dx = T(319.5);
				const T& dy = T(239.5);
				T predicted_x = T(693.8246970471848) * xp + dx;
				T predicted_y = T(693.8246970471848) * yp + dy;
				// The error is the difference between the predicted and observed position.
				residuals[0] = predicted_x - T(observed_x);
				residuals[1] = predicted_y - T(observed_y);
				// std::cout<<"error 0: "<<residuals[0]<<std::endl;
				// std::cout<<"error 1: "<<residuals[1]<<std::endl;

				return true;
			}
			// Factory to hide the construction of the CostFunction object from
			// the client code.
			static ceres::CostFunction* Create(const double observed_x,
				const double observed_y) {
					return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(
						new SnavelyReprojectionError(observed_x, observed_y)));
					}
					double observed_x;
					double observed_y;
};


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
					double* mutable_points()           { return parameters_  + 6 * num_cameras_; }
					double* mutable_camera_for_observation(int i) {
						return mutable_cameras() + camera_index_[i] * 6;
					}

					double* mutable_point_for_observation(int i) {
						return mutable_points() + point_index_[i] * 3;
					}


				bool LoadBA(OpenSfM* sfm){
						this->num_cameras_ = sfm->Z_i->back()+1;
						this->num_points_ = 0;
						this->num_observations_ = 0;
						for (size_t i = 0; i < (sfm->featureCell)->size(); i++) {
							/* code */
							if (  (*(sfm->featureCell))[i]->n_rows > 1 ){
								this->num_points_++;
								this->num_observations_ += (*(sfm->featureCell))[i]->n_rows;
							}
						}
						this->point_index_ = new int[num_observations_];
						this->camera_index_ = new int[num_observations_];
						this->observations_ = new double[2*num_observations_];
						this->num_parameters_ = 6 * num_cameras_ + 3 * num_points_;
						this->parameters_ = new double[num_parameters_];
						int n = 0;
						int pts_index = 0;
						for(int i = 0; i < (sfm->featureCell)->size(); i++){
							arma::fmat pts = *(*(sfm->featureCell))[i];
							// cout<<"pts: \n"<<pts<<endl;
							if ( pts.n_rows > 1 ){
								vector<unsigned> Z_index;
							for(int j = 0; j < (sfm->Z_j)->size(); j++)
								if( (*(sfm->Z_j))[j] == i && (Z_index.empty() || (*(sfm->Z_i))[j] != Z_index.back())){
								Z_index.push_back( (*(sfm->Z_i))[j]);
								//cout << "Zi" << (*(sfm->Z_i))[j] << endl;
								//cout << "Z: " << Z_index.back() << endl;
							}
							//  cout<<"Z_index.size(): "<<Z_index.size()<<endl;
								for(int j = 0; j < Z_index.size(); j++){
									//cout<<"Z_index[j]: "<<Z_index[j]<<endl;
									camera_index_[n] = int(Z_index[j]);
									point_index_[n] = pts_index;
									observations_[2 * n] = pts(j, 0);
									observations_[2 * n + 1] = pts(j, 1);

									// cout<<"n= "<<n<<endl;
									//cout << "ci" << n << " " << camera_index_[n] << endl;
									//cout << "pi" << n << " " <<  point_index_[n] << endl;
										n++;
								}
								pts_index++;

							}
						}
						// hopeflly it works
						for (size_t i = 0; i < (sfm->cameraPose)->size(); i++) {
							/* code */
							arma::fmat* tmp_pose = (*(sfm->cameraPose))[i];
							arma::fmat *R = new arma::fmat(3,3);
							*R = tmp_pose->submat(0,0,2,2);
							float* angles = new float[3];
							ceres::RotationMatrixToAngleAxis(R->memptr(),angles);
							parameters_[6 * i] = angles[0];
							parameters_[6 * i + 1] = angles[1];
							parameters_[6 * i + 2] = angles[2];
							parameters_[6 * i + 3] = tmp_pose->at(0, 3);
							parameters_[6 * i + 4] = tmp_pose->at(1, 3);
							parameters_[6 * i + 5] = tmp_pose->at(2, 3);
						}
					  n = 0;
						for(int i = 0; i < (sfm->featureCell)->size(); i++){
							arma::fmat pts = *(*(sfm->featureCell))[i];
							if ( pts.n_rows > 1 ){
								parameters_[6 * num_cameras_ + 3 * n] = double((sfm -> featureTable)->at<float>(i, 128));
								parameters_[6 * num_cameras_ + 3 * n + 1] = double( (sfm -> featureTable)->at<float>(i, 129));
								parameters_[6 * num_cameras_ + 3 * n + 2] = double( (sfm -> featureTable)->at<float>(i, 130));
								n++;
							}
						}
						//for(int i = 0; i < num_parameters_; i++)
						//	cout << "p" << i << "|" << parameters_[i] << endl;
						return true;
		}

int runBA(){
	const double* observations = this->observations();
	// cout<<observations[0]<<endl;
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
	// cout<< "this->num_observations(): "<< this->num_observations()<<endl;
  for (int i = 0; i < this->num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
		// cout<<" observation: "<<i <<" | "<<observations[2*i]<<endl;
    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0],
                                         observations[2 * i + 1]);
			ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
    problem.AddResidualBlock(cost_function,
                             loss_function /* squared loss */,
                             this->mutable_camera_for_observation(i),
                             this->mutable_point_for_observation(i));
  }
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
	// cout<<observations[0]<<endl;

  return 0;

		}

	int saveBA(OpenSfM* sfm){
		const double* observations_ = this->observations();
		int n = 0;
		for(int i = 0; i < (sfm->featureCell)->size(); i++){
			arma::fmat pts = *(*(sfm->featureCell))[i];
			if ( pts.n_rows > 1 ){
				(sfm -> featureTable)->at<float>(i, 128) = float(parameters_[6 * num_cameras_ + 3 * n]);
				(sfm -> featureTable)->at<float>(i, 129) = float(parameters_[6 * num_cameras_ + 3 * n + 1]);
				(sfm -> featureTable)->at<float>(i, 130) = float(parameters_[6 * num_cameras_ + 3 * n + 2]);
				n++;
			}
		}
		for(int i = 0; i < num_cameras_; i++){
			double* R = new double[9];
			ceres::AngleAxisToRotationMatrix<double>(parameters_+i*6,R);
			arma::mat R_1(reinterpret_cast<double*>(R),3,3);
      arma::mat T_1(reinterpret_cast<double*>(parameters_+i*6+3),3,1);
      R_1 = arma::join_rows(R_1,T_1);
			arma::fmat R_1_f = arma::conv_to<arma::fmat>::from(R_1);
			cout<<"diff Pose: "<<*(*(sfm->cameraPose))[i] -  R_1_f<<endl;
			*(*(sfm->cameraPose))[i] = R_1_f;
      R_1_f = (sfm->intrinsc_K)*R_1_f;
      *(*(sfm->camProjTable))[i] = R_1_f;
			delete[] R;
		}


	}

	private:
					int num_cameras_;
					int num_points_;
					int num_observations_;
					int num_parameters_;
					int* point_index_;
					int* camera_index_;
					double* observations_;
					double* parameters_;
};
#endif
