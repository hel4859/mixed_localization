#include "utils.h"

bool isZero(double num){
    return (num<0.0000001 and num>-0.0000001);
}

void constrainDegree(double &x){
    x = fmod(x + 180.0, 360.0);
    if (x < 0)
        x += 360.0;
    x -= 180.0;
}