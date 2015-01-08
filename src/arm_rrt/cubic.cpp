#include "schunk_rrt/cubic.h"

CubicSpline cubicSolve(const double &s0, const double &s1,\
                       double v0, double v1,\
                       const double &duration)
{
    CubicSpline cubic;
    cubic.d = s0;
    cubic.c = v0;
    //cubic.b = 3*(s1-s0)/pow(duration,2) - 2*v0/duration - v1/duration;
    //cubic.a = -2*(s1-s0)/pow(duration,3)+(v0+v1)/pow(duration,2);
    cubic.b = (3*s1-v1*duration-2*cubic.c*duration-3*cubic.d)/pow(duration,2);
    cubic.a = (s1-s0-cubic.c*duration-cubic.b*pow(duration,2))/pow(duration,3);
    return cubic;
}

double cubicValue(const CubicSpline &cubic, const double &t)
{
    //return (cubic.a*pow(t,3)+cubic.b*pow(t,2)+cubic.c*t+cubic.d);
    return (cubic.a*pow(t,3)+cubic.b*pow(t,2)+cubic.c*t+cubic.d);
}
