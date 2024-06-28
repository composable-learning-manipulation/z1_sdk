FROM ubuntu:20.04

ENV TZ=Europe/Moscow \
    DEBIAN_FRONTEND=noninteractive
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y g++ cmake git
RUN apt-get update && apt-get install -y libboost-all-dev libeigen3-dev

RUN mkdir /grab_data_itmo && cd /grab_data_itmo \
        && git clone https://github.com/unitreerobotics/z1_controller.git \
        && git clone https://github.com/composable-learning-manipulation/z1_sdk.git
RUN cd /grab_data_itmo/z1_controller && sed -i "$(($(wc -l < CMakeLists.txt)-2)),\$s/^/# /" CMakeLists.txt

RUN cd /grab_data_itmo/z1_controller && mkdir build && cd build && cmake ..  && make
RUN cd /grab_data_itmo/z1_sdk && mkdir build && cd build && cmake .. && make
