#include "fast_aligner.h"
#include "AndersonAcceleration.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <time.h>
#define SAME_THRESHOLD 1e-6


namespace proslam {
  FastAligner::FastAligner(AlignerParameters* parameters_) : BaseLocalMapAligner(parameters_) {
    //ds nothing to do
  }
  FastAligner::~FastAligner() {
    //ds nothing to do
  }

  void FastAligner::initialize(Closure* context_, const TransformMatrix3D& current_to_reference_) {
    //ds initialize base components
    _context              = context_;
    _current_to_reference = current_to_reference_;
    _parameters->damping = 0;
    _number_of_measurements = _context->correspondences.size();
    _errors.resize(_number_of_measurements);
    _inliers.resize(_number_of_measurements);

    //ds construct point cloud registration problem - compute landmark coordinates in local maps
    _information_matrix_vector.resize(_number_of_measurements);
    _moving.resize(_number_of_measurements);
    _fixed.resize(_number_of_measurements);
    const TransformMatrix3D& world_to_reference_local_map(context_->local_map_reference->worldToRobot());
    const TransformMatrix3D& world_to_query_local_map(context_->local_map_query->worldToRobot());
    for (Index u = 0; u < _number_of_measurements; ++u) {
      const Closure::Correspondence* correspondence = _context->correspondences[u];

      //ds point coordinates to register
      _fixed[u]  = world_to_reference_local_map*correspondence->reference->coordinates();
      _moving[u] = world_to_query_local_map*correspondence->query->coordinates();
      //ds set information matrix
      _information_matrix_vector[u].setIdentity();
      _information_matrix_vector[u] *= correspondence->matching_ratio;
    }
  }
  
  void FastAligner::linearize(const bool& ignore_outliers_) {
     //ds initialize setup
    _H.setZero();
    _b.setZero();
    _number_of_inliers  = 0;
    _number_of_outliers = 0;
    _total_error        = 0;

    //ds for all the points
    for (Index u = 0; u < _number_of_measurements; ++u) {
      _omega = _information_matrix_vector[u];

      //ds compute error based on items: local map merging
      const PointCoordinates sampled_point_in_reference   = _current_to_reference*_moving[u];
      const Vector3 error                                 = sampled_point_in_reference-_fixed[u];

      //ds update chi
      const real error_squared = error.transpose()*_omega*error;

      //ds check if outlier
      if (error_squared > _parameters->maximum_error_kernel) {
        _inliers[u] = false;
        ++_number_of_outliers;
        if (ignore_outliers_) {
          continue;
        }

        //ds proportionally reduce information value of the measurement
        _omega *= _parameters->maximum_error_kernel/error_squared;
      } else {
        _inliers[u] = true;
        ++_number_of_inliers;
      }
      _total_error += error_squared;

      //ds get the jacobian of the transform part = [I -2*skew(T*modelPoint)]
      _jacobian.block<3,3>(0,0).setIdentity();
      _jacobian.block<3,3>(0,3) = -2*srrg_core::skew(sampled_point_in_reference);

      //ds precompute transposed
      const Matrix6_3 jacobian_transposed(_jacobian.transpose( ));

      //ds accumulate
      _H += jacobian_transposed*_omega*_jacobian;
      _b += jacobian_transposed*_omega*error;
    }
  }

  void FastAligner::oneRound(const bool& ignore_outliers_) {
    //ds linearize system
    linearize(ignore_outliers_);

    //ds solve the system and update the estimate
    _current_to_reference = srrg_core::v2t(static_cast<const Vector6&>(_H.ldlt().solve(-_b)))*_current_to_reference;
    
     //ds enforce rotation symmetry
    const Matrix3 rotation   = _current_to_reference.linear();
    Matrix3 rotation_squared = rotation.transpose( )*rotation;
    rotation_squared.diagonal().array() -= 1;
    _current_to_reference.linear()      -= 0.5*rotation*rotation_squared;
  }

  void FastAligner::converge() {
    real total_error_previous = std::numeric_limits<real>::max();

    const int N = 3;
    int anderson_m = 5;

    MatrixNX X, Y;
    X.resize(3, _number_of_measurements);
    Y.resize(3, _number_of_measurements);
    VectorX W = VectorX::Zero(_number_of_measurements);

    for(Index i = 0; i < _number_of_measurements; i++){
        X(0, i) = _moving[i][0];
        X(1, i) = _moving[i][1];
        X(2, i) = _moving[i][2];
        Y(0, i) = _fixed[i][0];
        Y(1, i) = _fixed[i][1];
        Y(2, i) = _fixed[i][2];
    }

    //set Anderson Acc
    AndersonAcceleration accelerator_;
    AffineNd T = AffineNd::Identity();
    
    _current_to_reference = T;
    AffineNd SVD_T = T;

    //set initial distance
    for (Index i = 0; i<_number_of_measurements; i++) {
        Vector3 cur_p = T * X.col(i);
        W[i] = (cur_p - Y.col(i)).norm();
    }
    
    //init AA
    accelerator_.init(anderson_m, (N + 1) * (N + 1), LogMatrix(T.matrix()).data());
    /// start LS(FAST-ICP)
    for (Count iteration = 0; iteration < _parameters->maximum_number_of_iterations; ++iteration) {
        oneRound(false);

        if (_total_error < total_error_previous) {
        }
        else{
            accelerator_.replace(LogMatrix(SVD_T.matrix()).data());
            for (Count i = 0; i < _number_of_measurements; i++) {
                Vector3 cur_p = SVD_T * X.col(i);
                W[i] = (cur_p - Y.col(i)).norm();
            }
            oneRound(false);
        }
        
        // Rotation and translation update
        T = point_to_point(X, Y, W);

        //Anderson Acc
        SVD_T = T;
        AffineMatrixN Trans = (Eigen::Map<const AffineMatrixN>(accelerator_.compute(LogMatrix(T.matrix()).data()).data(), N+1, N+1)).exp();
        T.linear() = Trans.block(0,0,N,N);
        T.translation() = Trans.block(0,N,N,1);
        _current_to_reference = T;

        for (Index i = 0; i < _number_of_measurements; i++) {
                Vector3 cur_p = SVD_T * X.col(i);
                W[i] = (cur_p - Y.col(i)).norm();
        }

        if (_parameters->error_delta_for_convergence > std::fabs(total_error_previous-_total_error)) {

            //ds trigger inlier only runs
            oneRound(true);
            oneRound(true);
            oneRound(true);

            //ds system converged
            _has_system_converged = true;

            //ds compute inliers ratio
            const real inlier_ratio = static_cast<real>(_number_of_inliers)/_context->correspondences.size();

            //ds set out values
            _context->query_to_reference       = _current_to_reference;
            _context->icp_inlier_ratio         = inlier_ratio;
            _context->icp_number_of_inliers    = _number_of_inliers;
            _context->icp_number_of_iterations = iteration;

            //ds if the solution is acceptable
            if (_number_of_inliers > _parameters->minimum_number_of_inliers && inlier_ratio > _parameters->minimum_inlier_ratio) {
                LOG_INFO(std::printf("FASTAligner::converge|registered local maps [%06u:{%06u-%06u}] > [%06u:{%06u-%06u}] "
                                    "(correspondences: %3lu, iterations: %2u, inlier ratio: %5.3f, inliers: %2u)\n",
                _context->local_map_query->identifier(),
                _context->local_map_query->frames().front()->identifier(), _context->local_map_query->frames().back()->identifier(),
                _context->local_map_reference->identifier(),
                _context->local_map_reference->frames().front()->identifier(), _context->local_map_reference->frames().back()->identifier(),
                _context->correspondences.size(), iteration, inlier_ratio, _number_of_inliers))

                //ds enable closure
                _context->is_valid = true;

                //ds set inlier status
                for (Index u = 0; u < _number_of_measurements; ++u) {
                    _context->correspondences[u]->is_inlier = _inliers[u];
                }

                break;
            } else {
                LOG_DEBUG(std::printf("FASTAligner::converge|dropped registration for local maps [%06lu:{%06lu-%06lu}] > [%06lu:{%06lu-%06lu}] "
                                        "(correspondences: %3lu, iterations: %2lu, inlier ratio: %5.3f, inliers: %2lu)\n",
                _context->local_map_query->identifier(),
                _context->local_map_query->frames().front()->identifier(), _context->local_map_query->frames().back()->identifier(),
                _context->local_map_reference->identifier(),
                _context->local_map_reference->frames().front()->identifier(), _context->local_map_reference->frames().back()->identifier(),
                _context->correspondences.size(), iteration, inlier_ratio, _number_of_inliers))
                _context->is_valid = false;
                break;
            }
        } else {
            total_error_previous = _total_error;
        }
        if(iteration == _parameters->maximum_number_of_iterations-1) {
        _context->is_valid = false;
        _has_system_converged = false;
        LOG_DEBUG(std::cerr << "FASTAligner::converge|system did not converge - inlier ratio: " << static_cast<real>(_number_of_inliers)/_context->correspondences.size()
                            << " [" << _context->local_map_query->identifier() << "][" << _context->local_map_reference->identifier() << "]" << std::endl)
      }

    }   
  }

    AffineMatrixN FastAligner::LogMatrix(const AffineMatrixN& T){
        const int N = 3;

        Eigen::RealSchur<AffineMatrixN> schur(T);
        AffineMatrixN U = schur.matrixU();
        AffineMatrixN R = schur.matrixT();
        std::vector<bool> selected(N, true);
        MatrixNN mat_B = MatrixNN::Zero(N, N);
        MatrixNN mat_V = MatrixNN::Identity(N, N);

        for (int i = 0; i < N; i++)
        {
            if (selected[i] && fabs(R(i, i) - 1)> SAME_THRESHOLD)
            {
                int pair_second = -1;
                for (int j = i + 1; j <N; j++)
                {
                    if (fabs(R(j, j) - R(i, i)) < SAME_THRESHOLD)
                    {
                        pair_second = j;
                        selected[j] = false;
                        break;
                    }
                }
                if (pair_second > 0)
                {
                    selected[i] = false;
                    R(i, i) = R(i, i) < -1 ? -1 : R(i, i);
                    double theta = acos(R(i, i));
                    if (R(i, pair_second) < 0)
                    {
                        theta = -theta;
                    }
                    mat_B(i, pair_second) += theta;
                    mat_B(pair_second, i) += -theta;
                    mat_V(i, pair_second) += -theta / 2;
                    mat_V(pair_second, i) += theta / 2;
                    double coeff = 1 - (theta * R(i, pair_second)) / (2 * (1 - R(i, i)));
                    mat_V(i, i) += -coeff;
                    mat_V(pair_second, pair_second) += -coeff;
                }
            }
        }
        AffineMatrixN LogTrim = AffineMatrixN::Zero();
        LogTrim.block(0, 0, N, N) = mat_B;
        LogTrim.block(0, N, N, 1) = mat_V * R.block(0, N, N, 1);
        AffineMatrixN res = U * LogTrim * U.transpose();
        return res;
    }

    template <typename Derived1, typename Derived2, typename Derived3>
    AffineNd FastAligner::point_to_point(Eigen::MatrixBase<Derived1>& X,
                            Eigen::MatrixBase<Derived2>& Y,
                            const Eigen::MatrixBase<Derived3>& w) {
        int dim = X.rows();
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w / w.sum();
        /// De-mean
        Eigen::VectorXd X_mean(dim), Y_mean(dim);
        for (int i = 0; i<dim; ++i) {
            X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
            Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
        }
        X.colwise() -= X_mean;
        Y.colwise() -= Y_mean;

        /// Compute transformation
        AffineNd transformation;
        MatrixXX sigma = X * w_normalized.asDiagonal() * Y.transpose();
        Eigen::JacobiSVD<MatrixXX> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
        if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
            VectorN S = VectorN::Ones(dim); S(dim-1) = -1.0;
            transformation.linear() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
        }
        else {
            transformation.linear() = svd.matrixV()*svd.matrixU().transpose();
        }
        transformation.translation() = Y_mean - transformation.linear()*X_mean;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += Y_mean;
        /// Return transformation
        return transformation;
    }
}