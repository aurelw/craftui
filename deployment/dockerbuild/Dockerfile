FROM debian
MAINTAINER Aurel Wildfellner <aurel.w@gmail.com>

RUN apt-get update && \
    apt-get -y install usbutils \
        openni-utils \
        libopenni-sensor-primesense0 

RUN apt-get -y install \
	cmake \
	git \
	build-essential

RUN apt-get -y install \
	libboost-dev \
	libboost-system-dev \
	libboost-filesystem-dev \
	libboost-thread-dev \ 
	libboost-date-time-dev \
	libboost-iostreams-dev \
	libopenni-dev \
	libqhull-dev \
	libxt-dev \
	libeigen3-dev \
	libqt4-dev \
	libvtk5-dev \
	libflann-dev \
	libopencv-dev \
	libpcl-dev

RUN apt-get -y install \
	libzmq3-dev \
	libprotobuf-dev \
	protobuf-compiler \
	python-zmq
	

WORKDIR /root/
RUN git clone --branch develop https://github.com/aurelw/craftui.git
RUN mkdir craftui/build
WORKDIR craftui/build/
RUN cmake -DCMAKE_BUILD_TYPE="Release" ..
RUN make -j 4
WORKDIR bin/

