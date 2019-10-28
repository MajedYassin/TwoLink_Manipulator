
#include <gtest/gtest.h>
#include <cmath>


static float add_operation(float&& a){
    return sin(a) + cos(a);
}

TEST(Addition, add)
{
    float b = 1;
    ASSERT_EQ(add_operation(M_PI_2), b);
}

