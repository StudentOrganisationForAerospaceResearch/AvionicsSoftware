FROM ubuntu:18.10
ENV USER=root

# Set the initial working directory
WORKDIR /

RUN mkdir /AvionicsSoftware

# Copy the current directory contents into the container
VOLUME /AvionicsSoftware

WORKDIR /AvionicsSoftware

# Update apt-get
RUN apt-get update

# Install any needed packages
RUN apt-get install -y git
RUN apt-get install -y wget
RUN apt-get install -y lbzip2
RUN apt-get install -y build-essential
RUN apt-get install -y cmake
RUN apt-get install -y libusb-1.0.0-dev
RUN apt-get install -y pkg-config

WORKDIR /

RUN git clone https://github.com/adamgreen/gcc4mbed.git
WORKDIR /gcc4mbed
RUN ./linux_install
ENV PATH="/gcc4mbed/gcc-arm-none-eabi/bin:${PATH}"

WORKDIR /

RUN git clone https://github.com/texane/stlink.git
WORKDIR /stlink
RUN make release
RUN make debug
WORKDIR /stlink/build
RUN cmake -DCMAKE_BUILD_TYPE=Debug ..
RUN make
WORKDIR /stlink/build/Release
RUN make install
ENV PATH="/stlink/build:${PATH}"

WORKDIR /

RUN wget 'https://s3-us-west-2.amazonaws.com/ucsolarteam.hostedfiles/astyle'
RUN tar -zxvf astyle
WORKDIR /astyle/build/gcc
RUN make release
RUN make install

WORKDIR /

RUN echo 'alias flash="st-flash write ./build/AvionicsSoftware.bin 0x8000000"' >> ~/.bashrc
RUN echo 'alias erase="st-flash erase"' >> ~/.bashrc
Run echo 'alias clean="rm -r build"' >> ~/.bashrc
Run echo 'alias format="./format.sh"' >> ~/.bashrc

WORKDIR /AvionicsSoftware
