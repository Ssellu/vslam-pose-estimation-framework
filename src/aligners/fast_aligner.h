#pragma once
#include "base_local_map_aligner.h"

namespace proslam {

class FastAligner: public BaseLocalMapAligner, public AlignerWorkspace<6,3> {

PROSLAM_MAKE_PROCESSING_SUBCLASS(FastAligner, AlignerParameters)

public:
  virtual void initialize(Closure *context_, const TransformMatrix3D& current_to_reference_ = TransformMatrix3D::Identity());
  virtual void linearize(const bool& ignore_outliers_);
  virtual void oneRound(const bool& ignore_outliers_);
  virtual void converge();
protected:
  std::vector<Vector3> _moving;
private:
  AffineMatrixN LogMatrix(const AffineMatrixN& T);
  template <typename Derived1, typename Derived2, typename Derived3>
  AffineNd point_to_point(Eigen::MatrixBase<Derived1>& X,
                          Eigen::MatrixBase<Derived2>& Y,
                          const Eigen::MatrixBase<Derived3>& w);
};

typedef std::shared_ptr<FastAligner> FastAlignerPtr;

}