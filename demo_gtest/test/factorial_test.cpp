#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>

int factorial(int n)
{
    int ret = 1;
    for(int i=1;i<=n;i++)
        ret *= i;
    return ret;
}
TEST(FactorialTest,Negative)
{
    EXPECT_EQ(factorial(-5),1);
    EXPECT_GT(factorial(-10),0);
}
TEST(FactorialTest,Zero)
{
    EXPECT_EQ(factorial(0),1);
}
TEST(FactorialTest,Positive)
{
    EXPECT_EQ(factorial(2),2);
    EXPECT_EQ(factorial(3),6);
    EXPECT_EQ(factorial(8),40320);
}


int main(int, char**)
{
    testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
