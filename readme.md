# Opencv4.8 Example 

Trying out the newest opencv with the latest tools. Idea is to develop a set of examples with the latest opencv. Also make all the needed library environment for computer vision developers (especially SLAM) in one place with a docker. 

## Docker 
The supplied docker has opencv4.8 already setup and installed with graphics. 

You may generate a docker by yourself: 
```
docker build -t joke-image:with-opencv  
```

OR pull one from [docker hub](https://hub.docker.com/r/mpkuse/joke-image)
```
docker pull mpkuse/joke-image:with-opencv
```

## Run Docker
```
$(host) xhost +local:root 

$(host) docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/Downloads/opencv4.8_tutorial:/code mpkuse/joke-image:with-opencv-clean
```


## Compile and Run C++ Code (in docker)

```
$(docker) cd /code && mkdir build && cd build 
$(docker) cmake .. && make 
$(docker) ./example #should display a lena image
```


## Run Python code 
```
$(docker) cd py 
$(docker) python3 example_1.py
```
