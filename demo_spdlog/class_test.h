#pragma once

#include "spdlog_comm.h"

class class_test
{
private:
    /* data */
public:
    class_test(/* args */);
    ~class_test();
};

class_test::class_test(/* args */)
{
    SPD_INFO("info1");
    SPD_INFO("info2");
}

class_test::~class_test()
{
}
