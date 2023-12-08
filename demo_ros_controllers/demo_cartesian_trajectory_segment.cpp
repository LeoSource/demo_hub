#define _USE_MATH_DEFINES
#include <iostream>
#include "cartesian_trajectory_segment.h"

using namespace ros_controllers_cartesian;
const double R2D = 180.0/M_PI;
const double D2R = M_PI/180.0;

int main(int, char**)
{
    Eigen::Vector3d p,t1,t2;
    p<<0.5,0,0.2;
    t1<<0.3,0.1,0.1;
    t2<<-0.1,0.5,0.3;
    Eigen::Quaterniond q1 = Eigen::Quaterniond(sqrt(2)/2,0,sqrt(2)/2,0).normalized();
    std::cout<<q1*Eigen::Vector3d(0,0,10)<<std::endl;
    std::cout<<q1.inverse()*Eigen::Vector3d(0,0,10)<<std::endl;
    //--------------------------------------------------------------------------------
    // Cartesian -> Spline -> Cartesian
    //--------------------------------------------------------------------------------
    CartesianState c;

    c.q = Eigen::Quaterniond(Eigen::AngleAxisd(90*D2R,Eigen::Vector3d::UnitZ()));

    c.w.x() = 0;
    c.w.y() = 1;
    c.w.z() = 0;

    CartesianTrajectorySegment::SplineState s = convert(c);

    std::stringstream before;
    std::stringstream after;
    before << c;
    after << convert(convert(c));

    // The non-initialized case
    CartesianState d;
    before.clear();
    after.clear();
    before << d;
    after << convert(convert(d));
}
