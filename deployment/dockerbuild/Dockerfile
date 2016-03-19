FROM debian
MAINTAINER Aurel Wildfellner <aurel.w@gmail.com>

ENV PCL_DIR /root/pcl
ENV NUM_BUILD_PROCESSES 16

RUN apt-get update && \
    apt-get -y install usbutils \
        openni-utils \
        libopenni-sensor-primesense0 

RUN apt-get -y install \
	cmake \
	git \
	build-essential

RUN apt-get update && \
    apt-get -y install \
	libboost-dev \
	libboost-all-dev \
	python-dev \
	libpython-dev \
 	libusb-1.0-0-dev \
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
	libopencv-dev 

RUN apt-get -y install \
	libzmq3-dev \
	libprotobuf-dev \
	protobuf-compiler \
	python-zmq \
	python-protobuf
	

#### Build PCL ####
WORKDIR /root/
RUN mkdir -p $PCL_DIR && \
    git clone --branch pcl-1.8.0rc2 https://github.com/PointCloudLibrary/pcl.git $PCL_DIR
WORKDIR $PCL_DIR
RUN mkdir -p build
WORKDIR build
RUN cmake -DWITH_OPENNI=YES .. && make -j $NUM_BUILD_PROCESSES && make install

#### Build CraftUI ####
WORKDIR /root/
RUN git clone --branch develop https://github.com/aurelw/craftui.git
RUN mkdir craftui/build
WORKDIR craftui/build/
RUN cmake -DCMAKE_BUILD_TYPE="Release" ..
RUN make -j $NUM_BUILD_PROCESSES
WORKDIR bin/

#### Prepare Clients ####
RUN apt-get -y install \
	mosquitto-clients

