#include <cmath>

struct CubicSpline
{
    //f(x) = ax^3+bx^2+cx+d
    double a;
    double b;
    double c;
    double d;
};

CubicSpline cubicSolve(const double &s0, const double &s1,\
                       double v0, double v1,\
                       const double &duration);

double cubicValue(const CubicSpline &cubic, const double &t);
