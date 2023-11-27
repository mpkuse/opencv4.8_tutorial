cmake_minimum_required(VERSION 3.12)
project(YourOpenCVProject)

# Find the OpenCV package
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)


# Create the executable
add_executable(example_1 src/example_1.cpp )

# Include OpenCV headers
target_include_directories(example_1 PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link against OpenCV libraries
target_link_libraries(example_1 PRIVATE ${OpenCV_LIBS})


# set(TEST_SOURCES
#     tests/main.cpp
# )
file(GLOB TEST_SOURCES "tests/*.cpp")
set( TEST_LIBS 
)

add_executable(TestMyProject ${TEST_SOURCES})
target_link_libraries(TestMyProject ${GTEST_LIBRARIES} pthread ${TEST_LIBS} )