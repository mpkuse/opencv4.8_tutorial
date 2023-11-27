#include <gtest/gtest.h>

// A simple function to multiplication two numbers
int multiply(int a, int b) {
    return a * b;
}


// Test case to check the multiply function
TEST(MultiplyTest, PositiveNumbers) {
    EXPECT_EQ(multiply(2, 3), 6);
    EXPECT_EQ(multiply(8, 9), 72);
}

TEST(MultiplyTest, NegativeNumbers) {
    EXPECT_EQ(multiply(-2, -3), 6);
    EXPECT_EQ(multiply(-2, 3), -6);
    EXPECT_EQ(multiply(2, -3), -6);
}