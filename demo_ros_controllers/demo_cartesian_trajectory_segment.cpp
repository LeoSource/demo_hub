#include <iostream>
#include "cartesian_trajectory_segment.h"

using namespace ros_controllers_cartesian;

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

    c.v.x() = 1.1;
    c.v.y() = 1.2;
    c.v.z() = 1.3;

    c.w.x() = 2.1;
    c.w.y() = 2.2;
    c.w.z() = 2.3;

    c.v_dot.x() = 3.1;
    c.v_dot.y() = 3.2;
    c.v_dot.z() = 3.3;

    c.w_dot.x() = 4.1;
    c.w_dot.y() = 4.2;
    c.w_dot.z() = 4.3;

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
