
#include <eigen3/Eigen/Dense>

void pseudo_inverse(const Eigen::MatrixXf &M_, Eigen::MatrixXf &M_pinv_, bool damped)
{
    float lambda_max = damped ? 1.0e-2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType sing_vals_;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    sing_vals_ = svd.singularValues();
    Eigen::MatrixXf S_ = M_; // copying the dimensions of M_, its content is not needed.

    float lambda_quad = 0;
    float epsilon = 1e-4; //

    if ( sing_vals_(sing_vals_.size() - 1) < epsilon )
    {
        lambda_quad = ( 1 - std::pow( (sing_vals_(sing_vals_.size() - 1) / epsilon), 2) ) * std::pow(lambda_max, 2);
    }

    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
    {
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_quad);
    }

    M_pinv_ = Eigen::MatrixXf(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}
