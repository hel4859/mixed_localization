#ifndef EKF_H
#define EKF_H
#include <eigen3/Eigen/Eigen>
using namespace std;
class EKF_Pose {
public:
    //constructor
    EKF_Pose(){
        init_flag = false;
    }
    //destructor
    ~EKF_Pose(){}
    void init(const Eigen::Vector3d &X0, const Eigen::Matrix3d &P0);
    Eigen::Vector3d statePrediction(const double &travel_distance, const double &turn_radian, const Eigen::Matrix2d &Q);
    Eigen::Vector3d stateUpdate(const Eigen::Vector3d &Z, const Eigen::Matrix3d &R);
    Eigen::Matrix3d readP();
private:
    Eigen::Vector3d X;//x1:pos_x,x2:pos_y,x3:yaw
    Eigen::Matrix3d P;
    void constrainRadian(double &x);
    bool init_flag;
};
#endif
