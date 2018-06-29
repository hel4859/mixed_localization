//
// Created by wlh on 17-11-20.
//

#ifndef PROJECT_EKF_POSE2D_H
#define PROJECT_EKF_POSE2D_H

#include <eigen3/Eigen/Dense>
#include "kalman_filter/kalman_filter.h"

//states include: [ x, y, yaw ]

class EKF_Pose2D : public KalmanFilter {
public:

    /**
     * Constructor
     */
    EKF_Pose2D();

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void Predict(const Eigen::VectorXd &xp, const double& delta_t);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z);

    double x(){ return x_[0];}

    double y(){ return x_[1];}

    double yaw(){ return x_[2];}

private:
    void predict(const double& delta_t, const double& velocity, const double& yaw_rate, double& pose_x, double& pose_y, double& pose_yaw);

    void calJacobianA(const double& delta_t, const double& velocity, Eigen::Matrix3d& A);

    void calJacobianB(Eigen::Matrix<double,3,2>& B);
};

#endif //PROJECT_EKF_POSE2D_H
