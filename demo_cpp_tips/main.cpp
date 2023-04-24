#include "test_other.h"

int main(int, char**)
{
    // test_other::test_other();
    // test_other::test_fill();
    // test_other::convert_string();
    test_other::bind_function();
#ifdef SIMULATION
    std::cout<<"simulation mode"<<std::endl;
#else
    std::cout<<"runtime mode"<<std::endl;
#endif
}
