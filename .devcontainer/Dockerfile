# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.148.1/containers/cpp/.devcontainer/base.Dockerfile

# [Choice] Debian / Ubuntu version: debian-10, debian-9, ubuntu-20.04, ubuntu-18.04
ARG VARIANT="buster"
FROM mcr.microsoft.com/vscode/devcontainers/cpp:0-${VARIANT}
# FROM vsc-abb_egm_example-e081a7f64866868247c1fb37602a1c8b


# Install dependency packages.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive
RUN apt-get -y install --no-install-recommends \
    protobuf-compiler \
    libprotobuf-dev \
    libboost-regex-dev \
    libboost-system-dev \
    libboost-thread-dev 

# Get EGM library
RUN mkdir -p workspaces/
WORKDIR workspaces/
RUN git clone https://github.com/ros-industrial/abb_libegm.git && \ 
    cd abb_libegm && \
    git checkout 6613e3165ea68118c31f336ddbb7f970a7b0e489

# Build EGM librayr
RUN mkdir -p abb_libegm/build 
RUN cd abb_libegm/build && cmake .. && make

# Additional dependecies
RUN apt-get -y install --no-install-recommends \
    bash-completion \
    iputils-ping
