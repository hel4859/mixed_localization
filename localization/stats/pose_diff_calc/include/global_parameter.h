#ifndef GLOBAL_PARAMETER_H
#define GLOBAL_PARAMETER_H
    // unit: m, rad, etc.
    // positive direction of rotation: counterclockwise
    #include <cmath>
    using namespace std;
    const double EARTH_RAD_EQ = 1000 * 6378.137; //unit: m
    const double ODOMETRY_FACTOR = 0.0210386; //dist = pulse*odometry_factor, unit: m

    /*
    the parameters below this comment should be set very cautiously,
    their values show a significant impact on the performance of pose_fusion.
    */
    extern double SCALE; // used in map_frame.h

#endif // GLOBAL_PARAMETER_H
