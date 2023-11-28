#include <iostream>
#include "quintic_spline_segment.h"

using namespace trajectory_interface;

typedef QuinticSplineSegment<double> Segment;
typedef typename Segment::State State;
typedef typename Segment::Time  Time;

int main(int, char**)
{
    const unsigned int dim = 2;

    const Time start_time = 1.0;
    State start_state;
    start_state.position.push_back( 0.0);
    start_state.position.push_back(-1.0);

    const Time end_time = 2.0;
    State end_state;
    end_state.position.push_back( 1.0);
    end_state.position.push_back(-2.0);

    Segment segment(start_time, start_state, end_time, end_state);
    const Time duration = segment.endTime() - segment.startTime();

    State state;
    segment.sample(start_time+duration/2.0,state);

}
