#include "../include/ekf_pose.h"
void EKF_Pose::init(const Eigen::Vector3d  &X0, const Eigen::Matrix3d  &P0){
    X = X0;
    P = P0;
    init_flag = true;
}
Eigen::Vector3d EKF_Pose::statePrediction(const double  &travel_distance, const double  &turn_radian, const Eigen::Matrix2d  &Q){
    Eigen::Vector2d U;
    Eigen::Matrix3d A;// = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double,3,2> W;
    Eigen::Vector3d Buff;
    const double avg_yaw = X(2);

    U << travel_distance,turn_radian;
    A << 1, 0, -travel_distance*sin(avg_yaw),
         0, 1,  travel_distance*cos(avg_yaw),
         0, 0, 1;
    W << cos(avg_yaw), 0,
         sin(avg_yaw), 0,
         0, 1;
    Buff << U(0)*cos(avg_yaw), U(0)*sin(avg_yaw), U(1);
    if (turn_radian>0.00001)
    {
        //CTRV model
        X(0) = X(0) + travel_distance / turn_radian * ( sin(avg_yaw + turn_radian) - sin(avg_yaw));
        X(1) = X(1) + travel_distance / turn_radian * ( cos(avg_yaw) - cos(avg_yaw + turn_radian));
        X(2) = X(2) + U(1);
    }
    else
        //linear model
        X = X + Buff;
    P = A * P * A.transpose() + W * Q * W.transpose();
    if (std::isnan(X(0))){
        P << 10000,0,0,
              0,10000,0,
              0,0,10000;
        X << 0, 0, 0;
    }
    constrainRadian(X(2));
    return X;
}
Eigen::Vector3d EKF_Pose::stateUpdate(const Eigen::Vector3d  &Z, const Eigen::Matrix3d  &R){
    const Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d S = H * P * H.transpose() + R;
    const Eigen::Matrix3d K = P * H.transpose() * S.inverse();
    Eigen::Vector3d Y = Z - H * X;
    constrainRadian(Y(2));
    X = X + K * Y;
    P = (I - K * H) * P;
    if (std::isnan(P(0,0))){
        P << 10000,0,0,
              0,10000,0,
              0,0,10000;
        X << 0, 0, 0;
    }
    constrainRadian(X(2));
    return X;
}
Eigen::Matrix3d EKF_Pose::readP(){
    return P;
}
void EKF_Pose::constrainRadian(double &x){
    x = fmod(x + M_PI,2 * M_PI);
    if (x < 0)
        x += 2*M_PI;
    x -= M_PI;
}
