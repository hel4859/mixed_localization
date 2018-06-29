//
// Created by wlh on 17-11-20.
//

#include <iostream>
#include "kalman_filter/ekf_pose2d.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

EKF_Pose2D::EKF_Pose2D()
{
    is_initialized_ = false;
}


void EKF_Pose2D::Predict(const Eigen::VectorXd &xp, const double &delta_t)
{
    // x_ = [ pose_x, pose_y, pose_yaw]
    // xp = [ velocity, yaw_rate ]

    // update time stamp
    time_stamp_ += delta_t;

    double velocity = xp[0];
    double yaw_rate = xp[1];

    double d_inc = velocity * delta_t;
    double yaw_inc = yaw_rate * delta_t;

    double &pose_x = x_[0];
    double &pose_y = x_[1];
    double &pose_yaw = x_[2];

    Eigen::Matrix3d A;
    Eigen::Matrix<double,3,2> B;

    A << 1, 0, -d_inc*sin(pose_yaw),
            0, 1,  d_inc*cos(pose_yaw),
            0, 0, 1;

    B << cos(pose_yaw), 0,
            sin(pose_yaw), 0,
            0, 1;


    pose_x += d_inc * cos( pose_yaw );
    pose_y += d_inc* sin( pose_yaw );
    pose_yaw += yaw_inc;

    NormalizeAngle(pose_yaw);


    double std_noise_encoder = 0.03 * sqrtf(delta_t);
    double std_noise_imu = 0.01 * sqrtf(delta_t);
    Q_ << pow(std_noise_encoder,2),0,
            0,pow(std_noise_imu,2);

    P_ = A * P_ * A.transpose() + B * Q_ * B.transpose();

//    std::cout<<"std_noise_encoder: "<<std_noise_encoder<<"\tstd_noise_imu: "<<std_noise_imu<<std::endl;
//    std::cout<<"Q_: "<<std::endl<<Q_<<std::endl;
//    std::cout<<"P_: "<<std::endl<<P_<<std::endl;
//    std::cout<<"B: "<<std::endl<<B<<std::endl;


//    Eigen::Matrix3d A;
//    Eigen::Matrix<double,3,2> B;
//
//    calJacobianA(delta_t, xp[0], A);
//    calJacobianB(B);
//
//    // predict x_
//    NormalizeAngle(x_[2]);
//    predict(delta_t, xp[0], xp[1], x_[0], x_[1], x_[2]);
//    NormalizeAngle(x_[2]);
//
//    // predict P_
//
//    double std_noise_encoder = 0.01 * sqrtf(delta_t);
//    double std_noise_imu = 0.005 * sqrtf(delta_t);
//    Q_ << pow(std_noise_encoder,2),0,
//            0,pow(std_noise_imu,2);
//
//    P_ = A * P_ * A.transpose() + B * Q_ * B.transpose();


}

void EKF_Pose2D::Update(const Eigen::VectorXd &z)
{
    VectorXd z_pred = H_ * x_;
    NormalizeAngle(z_pred[2]);

//    std::cout<<"zpred: "<<std::endl<<z_pred<<std::endl;

    VectorXd y = z - z_pred;
    NormalizeAngle(y[2]);

//    std::cout<<"y: "<<std::endl<<y<<std::endl;

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S.inverse();
    MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());

//    std::cout<<"S: "<<std::endl<<S<<std::endl;
//    std::cout<<"K: "<<std::endl<<K<<std::endl;
//    std::cout<<"R: "<<std::endl<<R_<<std::endl;
    // update x_
    x_ = x_ + K * y;

    NormalizeAngle(x_[2]);

    // update P_
    P_ = (I - K * H_) * P_;

//    std::cout<<"x_: "<<std::endl<<x_<<std::endl;
//    std::cout<<"P_: "<<std::endl<<P_<<std::endl;
}

void EKF_Pose2D::predict(const double &delta_t, const double &velocity, const double &yaw_rate,  double &pose_x,
                         double &pose_y, double& pose_yaw)
{
    double d_inc = velocity * delta_t;
    double yaw_inc = yaw_rate * delta_t;

//    std::cout<<"pose_x: "<<pose_x<<"\tpose_y: "<<pose_y<<"\tpose_yaw: "<<pose_yaw<<std::endl;
//    std::cout<<"d_inc: "<<d_inc<<"\tyaw_inc: "<<yaw_inc<<std::endl;

//    if (fabs( yaw_rate ) < 0.0001) {
        pose_x += d_inc * cos( pose_yaw );
        pose_y += d_inc* sin( pose_yaw );
        pose_yaw += yaw_inc;
//    }
//    else{
//        pose_x += ( d_inc / yaw_inc ) * ( sin( pose_yaw + yaw_inc ) - sin( pose_yaw ) );
//        pose_y += ( d_inc / yaw_inc ) * ( cos( pose_yaw )  - cos( pose_yaw + yaw_inc ) );
//        pose_yaw += yaw_inc;
//    }
}

void EKF_Pose2D::calJacobianA(const double &delta_t, const double &velocity, Eigen::Matrix3d& A)
{
    /*
        System transformation matrix: X[k+1] = A * X[k] + B * U[k]
        Nonlinear form:               X[k+1] = f(X[k],U[k])

        Discretized form:

        | x[k+1]  |   | x[k] + d[k+1] * cos( θ[k] ) |   | f1 |
        | y[k+1]  | = | y[k] + d[k+1] * sin( θ[k] ) | = | f2 |
        | θ[k+1]  |   |     θ[k+1] + δθ[k]          |   | f3 |

        Calculate Jacobian matrix for A

        | f1'(x), f1'(y), f1'(θ) |   | 1,  0,  -d[k+1]*sin(θ[k]) |
        | f2'(x), f2'(y), f2'(θ) | = | 0,  1,   d[k+1]*sin(θ[k]) |
        | f3'(x), f3'(y), f3'(θ) |   | 0,  0,        1           |
    */
    double d_inc = velocity * delta_t;
    double pose_yaw = x_[2];
    A << 1, 0, -d_inc*sin(pose_yaw),
            0, 1,  d_inc*cos(pose_yaw),
            0, 0, 1;

}

void EKF_Pose2D::calJacobianB(Eigen::Matrix<double,3,2>& B)
{
    /*
        System transformation matrix: X[k+1] = A * X[k] + B * U[k]
        Nonlinear form:               X[k+1] = f(X[k],U[k])

        Discretized form:

        | x[k+1]  |   | x[k] + d[k+1] * cos( θ[k] ) |   | f1 |
        | y[k+1]  | = | y[k] + d[k+1] * sin( θ[k] ) | = | f2 |
        | θ[k+1]  |   |     θ[k+1] + δθ[k]          |   | f3 |

        Calculate Jacobian matrix for B

        | f1'(d), f1'(θ) |   | cos(θ[k]),  0 |
        | f2'(d), f2'(θ) | = | sin(θ[k]),  0 |
        | f3'(d), f3'(θ) |   |    0     ,  1 |
    */

    double pose_yaw = x_[2];
    B << cos(pose_yaw), 0,
        sin(pose_yaw), 0,
        0, 1;
}
