#include <gtest/gtest.h>

// Define a fixture class
class AddTestFixture : public ::testing::Test {
protected:
    // Set up common resources for the test cases
    void SetUp() override {
        // Any setup code you need for your tests goes here
    }

    // Tear down common resources for the test cases
    void TearDown() override {
        // Any cleanup code you need for your tests goes here
    }
};

// A simple function to add two numbers
int add(int a, int b) {
    return a + b;
}


// Test case to check the add function
TEST(AddTest, PositiveNumbers) {
    EXPECT_EQ(add(2, 3), 5);
}

TEST(AddTest, NegativeNumbers) {
    EXPECT_EQ(add(-2, -3), -5);
}

// Test case using the fixture
TEST_F(AddTestFixture, PositiveNumbers) {
    EXPECT_EQ(add(2, 3), 5);
}

TEST_F(AddTestFixture, NegativeNumbers) {
    EXPECT_EQ(add(-2, -3), -5);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}