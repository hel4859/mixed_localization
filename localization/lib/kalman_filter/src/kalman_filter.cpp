#include "kalman_filter.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter()
{
    is_initialized_ = false;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::InitFilter(const double& time, VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

  time_stamp_ = time;
  is_initialized_ = true;
}

void KalmanFilter::Predict(const Eigen::VectorXd &xp, const double& delta_t) {
  /**
  TODO:
    * predict the state
  */
    time_stamp_ += delta_t;

    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S.inverse();
    MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
    x_ = x_ + K * y;
    P_ = (I - K * H_) * P_;
}

