cmake_minimum_required(VERSION 3.12)
project(YourOpenCVProject)

# Find the OpenCV package
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

# Create the executable
add_executable(example_1 src/example_1.cpp) #TODO: remove? 

# Include OpenCV headers
target_include_directories(example_1 PRIVATE ${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Link against OpenCV libraries
target_link_libraries(example_1 PRIVATE ${OpenCV_LIBS})

set(TEST_SOURCES
    tests/main.cpp
    tests/test_multiply.cpp
    tests/test_opencv_examples.cpp
    tests/test_opencv_contrib.cpp 
)
#file(GLOB TEST_SOURCES "tests/*.cpp")

# Check if OpenCV was built with opencv_contrib
if(OpenCV_CONTRIB_WORLD)
    message(STATUS "OpenCV was built with opencv_contrib modules.")
else()
    message(STATUS "OpenCV was not built with opencv_contrib modules.")

    #remove opencv test which uses contrib
    list(REMOVE_ITEM TEST_SOURCES "tests/test_opencv_contrib.cpp")
endif()



add_executable(TestMyProject ${TEST_SOURCES})
target_link_libraries(TestMyProject ${GTEST_LIBRARIES} pthread  ${OpenCV_LIBS})