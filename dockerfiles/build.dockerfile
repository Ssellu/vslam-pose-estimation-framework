FROM osrf/ros:kinetic-desktop-full

MAINTAINER Seungho

LABEL org.opencontainers.image.authors="slkumquat@gmail.com"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN echo "== Install Basic Tools =="" && \
  apt-get install git -y && \
  apt-get install sudo -y && \
  apt-get install wget -y && \
  apt-get install ninja-build -y \
