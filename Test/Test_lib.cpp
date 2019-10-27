
#include <gtest/gtest.h>


static int add_operation(int&& a, int&& b){
    return a + b;
}

TEST(Addition, add){
    int c = 5;
    ASSERT_EQ(add_operation(2, 3),  c);
}

