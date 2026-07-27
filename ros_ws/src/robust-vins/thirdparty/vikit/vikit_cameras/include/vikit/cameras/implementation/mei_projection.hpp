#include <iostream>
#include <glog/logging.h>
#include <vikit/cameras/mei_projection.h>

namespace vk {
namespace cameras {

namespace dis {

inline void distortionCataCamera(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u, const Eigen::Vector4d &D)
{
  double k1 = D(0);
  double k2 = D(1);
  double p1 = D(2);
  double p2 = D(3);

  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
      p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

inline void liftProjectiveCataCamera(const Eigen::Vector2d &p, Eigen::Vector3d &P,
                                      Eigen::Matrix3d &K, const Eigen::Vector4d &D, 
                                      bool m_noDistortion, double xi_param)
{
  double mx_d, my_d, mx_u, my_u;
  double rho2_d;

  double gamma1 = K(0, 0);
  double gamma2 = K(1, 1);
  double u0 = K(0, 2);
  double v0 = K(1, 2);

  double m_inv_K11 = 1.0 / gamma1;
  double m_inv_K13 = -u0 / gamma1;
  double m_inv_K22 = 1.0 / gamma2;
  double m_inv_K23 = -v0 / gamma2;

  // Lift points to normalised plane
  mx_d = m_inv_K11 * p(0) + m_inv_K13;
  my_d = m_inv_K22 * p(1) + m_inv_K23;

  if (m_noDistortion)
  {
    mx_u = mx_d;
    my_u = my_d;
  }
  else
  {

    // Recursive distortion model
    int n = 8;
    Eigen::Vector2d d_u;
    distortionCataCamera(Eigen::Vector2d(mx_d, my_d), d_u, D);
    // Approximate value
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);

    for (int i = 1; i < n; ++i)
    {
      distortionCataCamera(Eigen::Vector2d(mx_u, my_u), d_u, D);
      mx_u = mx_d - d_u(0);
      my_u = my_d - d_u(1);
    }
  }

  // Obtain a projective ray
  double xi = xi_param;
  if (xi == 1.0)
  {
    P << mx_u, my_u, (1.0 - mx_u * mx_u - my_u * my_u) / 2.0;
  }
  else
  {
    // Reuse variable
    rho2_d = mx_u * mx_u + my_u * my_u;
    P << mx_u, my_u, 1.0 - xi * (rho2_d + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * rho2_d));
  }
}

inline void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p, const Eigen::Vector4d &D, double xi, 
                         double fx, double fy, double cx, double cy) 
{
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    double z = P(2) + xi * P.norm();
    p_u << P(0) / z, P(1) / z;

    // if (m_noDistortion)
    // {
    //     p_d = p_u;
    // }
    // else
    // {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortionCataCamera(p_u, d_u, D);
        p_d = p_u + d_u;
    // }

    // Apply generalised projection matrix
    p << fx * p_d(0) + cx,
         fy * p_d(1) + cy;
}

}

template<typename Distortion>
MeiProjection<Distortion>::MeiProjection(
    double fx, double fy, double cx, double cy, distortion_t distortion)
  : fx_(fx), fy_(fy), fx_inv_(1.0/fx_), fy_inv_(1.0/fy_)
  , cx_(cx), cy_(cy), distortion_(distortion)
{}

template<typename Distortion>
MeiProjection<Distortion>::MeiProjection(
    const Eigen::VectorXd& intrinsics, distortion_t distortion)
  : distortion_(distortion)
{
  CHECK(intrinsics.size() == 5);
  xi_ = intrinsics(0);
  fx_ = intrinsics(1);
  fy_ = intrinsics(2);
  cx_ = intrinsics(3);
  cy_ = intrinsics(4);
  fx_inv_ = 1.0/fx_;
  fy_inv_ = 1.0/fy_;
}

template<typename Distortion>
bool MeiProjection<Distortion>::backProject3(
    const Eigen::Ref<const Eigen::Vector2d>& keypoint,
    Eigen::Vector3d* out_point_3d) const
{
  // double x = (keypoint[0]-cx_)*fx_inv_;
  // double y = (keypoint[1]-cy_)*fy_inv_;
  // distortion_.undistort(x, y);
  // (*out_point_3d)[0] = x;
  // (*out_point_3d)[1] = y;
  // (*out_point_3d)[2] = 1.0;

  Eigen::VectorXd D = getDistortionParameters();
  Eigen::Matrix3d K;
  K(0,0) = fx_;
  K(1,1) = fy_;
  K(0,2) = cx_;
  K(1,2) = cy_;
  Eigen::Vector3d out_p;
  dis::liftProjectiveCataCamera(keypoint, out_p, K, D, false, xi_);
  double z = out_p.z();
  (*out_point_3d)[0] = out_p.x() / z;
  (*out_point_3d)[1] = out_p.y() / z;
  (*out_point_3d)[2] = out_p.z() / z;

  return true;
}

template<typename Distortion>
void MeiProjection<Distortion>::project3(
    const Eigen::Ref<const Eigen::Vector3d>& point_3d,
    Eigen::Vector2d* out_keypoint,
    Eigen::Matrix<double, 2, 3>* out_jacobian_point) const
{
  // const double z_inv = 1 / point_3d(2);
  // // TODO(tcies) precalced member?
  // const Eigen::DiagonalMatrix<double, 2> focal_matrix(fx_, fy_);
  // const Eigen::Vector2d uv = point_3d.head<2>() * z_inv;
  // (*out_keypoint) =
  //     focal_matrix * distortion_.distort(uv) + Eigen::Vector2d(cx_, cy_);

  // if(out_jacobian_point)
  // {
  //   Eigen::Matrix<double, 2, 3> duv_dxy;
  //   duv_dxy.leftCols<2>() = Eigen::Matrix2d::Identity() * z_inv;
  //   duv_dxy.rightCols<1>() = - point_3d.head<2>() * z_inv * z_inv;

  //   (*out_jacobian_point) = focal_matrix * distortion_.jacobian(uv) * duv_dxy;
  // }

  Eigen::VectorXd D = getDistortionParameters();
  Eigen::Vector2d out_p;
  dis::spaceToPlane(point_3d, out_p, D, xi_, fx_, fy_, cx_, cy_);
  *out_keypoint = out_p;
}

template<typename Distortion>
double MeiProjection<Distortion>::errorMultiplier() const
{
  return std::abs(fx_);
}

template<typename Distortion>
double MeiProjection<Distortion>::getAngleError(double img_err) const
{
  return std::atan(img_err/(2.0*fx_)) + std::atan(img_err/(2.0*fy_));
}

template<typename Distortion>
MeiProjection<Distortion>
MeiProjection<Distortion>::createTestProjection(
    const size_t image_width, const size_t image_height) {
  return MeiProjection(
      image_width / 2, image_width / 2, image_width / 2, image_height / 2,
      Distortion::createTestDistortion());
}

template<typename Distortion>
template<typename T>
const T* MeiProjection<Distortion>::distortion() const
{
  return dynamic_cast<const T*>(&distortion_);
}

template<typename Distortion>
void MeiProjection<Distortion>::print(std::ostream& out) const
{
  out << "  Projection = MEI" << std::endl;
  out << "  Xi = (" << xi_ << ")" << std::endl;
  out << "  Focal length = (" << fx_ << ", " << fy_ << ")" << std::endl;
  out << "  Principal point = (" << cx_ << ", " << cy_ << ")" << std::endl;
  distortion_.print(out);
}

template<typename Distortion>
Eigen::VectorXd MeiProjection<Distortion>::getIntrinsicParameters() const
{
  Eigen::VectorXd intrinsics(5);
  intrinsics(0) = fx_;
  intrinsics(1) = fx_;
  intrinsics(2) = fy_;
  intrinsics(3) = cx_;
  intrinsics(4) = cy_;
  return intrinsics;
}

template<typename Distortion>
Eigen::VectorXd MeiProjection<Distortion>::getDistortionParameters() const
{
  return distortion_.getDistortionParameters();
}

} // namespace cameras
} // namespace vk
