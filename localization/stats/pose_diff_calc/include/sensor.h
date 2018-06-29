#ifndef SENSOR_H
#define SENSOR_H
#include <string>
using namespace std;
class Sensor{
public:
    const string node_name;
    const double T;
    Sensor(ros::NodeHandle &node,
           const string &name,
           const double &period):n(node),node_name(name),T(period){}
    ~Sensor(){}
protected:
    ros::NodeHandle &n;
};

#endif // SENSOR_H
