cmake_minimum_required(VERSION 3.12)
project(YourOpenCVProject)

# Find the OpenCV package
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)



# Build helpers library 
add_library(helpers SHARED
    src/helpers/MiscUtils.cpp
    src/helpers/TermColors.cpp
    src/helpers/RawFileIO.cpp
)

# Build geom library 
add_library(geom SHARED
    src/geom/ImagedFeatures.cpp
    src/geom/PairMatcher.cpp
    src/geom/TwoViewGeometry.cpp
)

# include directory 
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})


# Create the executable
add_executable(example_1 src/example_1.cpp) #TODO: remove? 
target_include_directories(example_1 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(example_1 PRIVATE ${OpenCV_LIBS})


# Tests 
set(TEST_SOURCES
    tests/main.cpp
    tests/test_multiply.cpp
    tests/test_opencv_examples.cpp
    tests/test_opencv_contrib.cpp 
    tests/test_opencv_pose_computation.cpp
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
target_link_libraries(TestMyProject ${GTEST_LIBRARIES} pthread  ${OpenCV_LIBS} helpers geom)