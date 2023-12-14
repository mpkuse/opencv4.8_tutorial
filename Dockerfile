FROM ubuntu:22.04

WORKDIR /app

# Build Essentials 
RUN apt-get update && apt-get install -y git build-essential cmake  \
    && rm -rf /var/lib/apt/lists/*

# Cmake Dependencies 
RUN apt-get update && apt-get install -y libeigen3-dev libspdlog-dev libsuitesparse-dev
RUN apt-get update && apt-get install -y qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5


ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH


#---- Clone Opencv4.5 with contrib and build it 
# see: https://docs.opencv.org/4.x/db/d05/tutorial_config_reference.html for more options
#RUN apt-get update && apt-get install -y wget unzip 

#---- Python3 
RUN apt-get update \
    && apt-get install -y python3-pip && pip3 install numpy 

#---- libgtk (for opencv highgui)
RUN apt-get update \
    && apt-get install -y libgtk-3-dev 

#---- ffmpeg (video io in opencv)
RUN apt-get update \
    && apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libavutil-dev libavcodec-extra

#---- viz dependencies: VTK (comment these, if not compiling with contrib)
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_PRIORITY=critical
RUN apt-get update \
    && apt-get install -y libvtk9-qt-dev 


#---- Clone Opencv4.8
RUN git clone -q --depth 1 --branch 4.8.0 https://github.com/opencv/opencv \
   && git clone -q --depth 1 --branch 4.8.0 https://github.com/opencv/opencv_contrib \
   && mkdir opencv-build && cd opencv-build \
   && cmake  -DCMAKE_CXX_STANDARD=17 -DWITH_VTK=ON -DOPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ../opencv \
   && make && make install \
   && cd .. && rm -rf opencv opencv-build opencv_contrib 

# OpenCV 4.8 only 
#RUN git clone -q --depth 1 --branch 4.8.0 https://github.com/opencv/opencv \
#    && mkdir opencv-build && cd opencv-build && cmake  -DCMAKE_CXX_STANDARD=17 ../opencv && make && make install && rm -rf opencv opencv-build 

ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages

#---- Gtests 
RUN apt-get update \
    && apt-get install -y libgtest-dev


#---- Clone g2o and Build it 
RUN git clone -q --depth 1 --branch 20230806_git https://github.com/RainerKuemmerle/g2o \
    && cd g2o \
    && mkdir build && cd build && cmake .. && make && make install 


#-----------------------------
# How to build docker image  #
#-----------------------------
# docker build -t joke-image .
# docker commit <CONTAINER_ID> joke-image:taag
