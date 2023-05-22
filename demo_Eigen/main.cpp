#include "test_eigen.h"

int main(int, char**) 
{
    
    //// test transformation between eigen matrix and array/vector
    //array2eigenMat();
    //eigenMat2array();

    //array2eigenVec();
    //eigenVec2array();

    //vector2eigenMat();
    //eigenMat2vector();

    //vector2eigenVec();
    //eigenVec2vector();

    test_eigen::test_eigen();
    // test_eigen::test_lambda();
    // test_eigen::test_eigen_error();
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    
}
