# ncs-lane-following-demo
***

# rpi-ncsdk-docker
_In this tutorial we're going to build Intel [Intel® Movidius™ Neural Compute SDK](https://github.com/movidius/ncsdk) docker image for Raspberry Pi on x86 machine._

1. First, install cross-compiling tool for building arm image on x86 machine. `./qemu-arm-static` is provided for Ubuntu 18.04, but if this does not work, run 

```sh
git clone --recursive git@github.com:duckietown/rpi-ncsdk-docker.git && \
    cd rpi-ncsdk-docker && \
    sudo apt-get install qemu-user-static && \
    cp /usr/bin/qemu-arm-static .
```

2. To build new docker image, run this command bellow on your favorite terminal

```sh
docker build -t andyser/ncs_lane_following .
```

## How use this docker image
First plug in an NCS and run the following command on your duckiebot

`
duckiebot $ docker run -it --name ncs_following --net host --privileged -v /dev:/dev -v /data:/data allenou/ncs_following_demo
`

## Image reference
This docker image is built from a github project called [ncs_lane_following](https://github.com/ARG-NCTU/ncs_lane_following) under ARG-NCTU, which is made by [Peter Hung](https://github.com/losttime1001)

#TODO model explanation

#TODO model visualization

#TODO demo video

[![Docker Build Status](https://img.shields.io/docker/build/allenou/ncs_following_demo.svg)](https://hub.docker.com/r/allenou/ncs_following_demo/builds)
