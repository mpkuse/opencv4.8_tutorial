# Opencv4.5 Example 

Trying out the newest opencv with the latest tools 

## Docker 
The supplied docker has opencv4.8 already setup and installed with graphics. 

You may generate a docker by yourself: 
```
docker build -t joke-image:with-opencv  
```

OR pull one from docker hub 
```
docker pull mpkuse/joke-image:with-opencv
```

## Run this repo with Docker
```
$(host) xhost +local:root 

$(host) docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/Downloads/opencv4.8_tutorial:/code joke-image:with-opencv

$(docker) cd /code && mkdir build && cd build 
$(docker) cmake .. && make 
$(docker) ./example #should display a lena image
```


