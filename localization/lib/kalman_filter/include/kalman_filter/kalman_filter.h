#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <eigen3/Eigen/Dense>

class KalmanFilter {

protected:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  // init flag
  bool is_initialized_;

  double time_stamp_;

public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  virtual void InitFilter(const double& time, Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  virtual void Predict(const Eigen::VectorXd &xp, const double& delta_t);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  virtual void Update(const Eigen::VectorXd &z);


  Eigen::VectorXd State(){ return x_; }

  Eigen::VectorXd StateCovariance(){ return P_; }

  void ResetState(const Eigen::VectorXd& x){ x_ = x; }

  void ResetStateCovariance(const Eigen::MatrixXd& P){ P_ = P; }

  void SetMeasurementCovariance(const Eigen::MatrixXd& R){ R_ = R; }

  double Time(){ return time_stamp_; }

  bool IsInited(){ return is_initialized_; }

  void NormalizeAngle(double& angle_rad){
      while (angle_rad >= M_PI) angle_rad -= 2 * M_PI;
      while (angle_rad < -M_PI) angle_rad += 2 * M_PI;
  }

};

#endif /* KALMAN_FILTER_H_ */
