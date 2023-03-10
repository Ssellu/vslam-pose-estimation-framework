FROM osrf/ros:kinetic-desktop-full

LABEL org.opencontainers.image.authors="slkumquat@gmail.com"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN echo "== Install Basic Tools ==" && \
    # Related to build
    apt-get install build-essential -y && \
    apt-get install cmake -y && \
    apt-get install git -y && \
    apt-get install sudo -y && \
    apt-get install wget -y && \
    apt-get install ninja-build && \
    apt-get install vim -y && \
    apt-get install python3 -y && \
    apt-get install python3-pip -y && \
    # Related to JetBrains CLion Docker develpoment...
    apt-get install -y ssh && \
    apt-get install -y gcc && \
    apt-get install -y g++ && \
    apt-get install -y gdb && \
    apt-get install -y clang && \
    apt-get install -y rsync && \
    apt-get install -y tar && \
    apt-get install -y mesa-utils && \
    # Related to X11 remote disply
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y libglu1-mesa-dev && \
    apt-get install -y mesa-common-dev && \
    apt-get install -y x11-utils && \
    apt-get install -y x11-apps


RUN echo "== Install ProSLAM Prerequisites == " && \
    apt-get install libeigen3-dev -y && \
    apt-get install libsuitesparse-dev  -y && \
    apt-get install freeglut3-dev  -y && \
    apt-get install libqglviewer-dev  -y && \
    apt-get install libyaml-cpp-dev -y
