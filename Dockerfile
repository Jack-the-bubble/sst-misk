FROM ros:noetic
LABEL maintainer="adam.krawczyk@husarion.com"
# we want to be able to source
SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update 
RUN apt-get install -y git 
RUN apt-get install -y libssl-dev
RUN apt-get update && apt-get install -y -qq apt-utils
RUN apt-get install -y -qq dirmngr 
RUN apt-get install -y -qq apt-transport-https software-properties-common 
RUN apt-get install -y -qq lsb-release gnupg gnupg-l10n gnupg-utils gpg gpg-agent gpg-wks-client gpg-wks-server gpgconf gpgsm gpgv dialog nano vim wget curl unzip gnupg2 
RUN apt-get install -y -qq ca-certificates 
RUN apt-get update 
RUN apt-get install -y -qq python3-pip


RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y ros-noetic-tf ros-noetic-tf2-geometry-msgs

RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc

# install stage 
RUN apt-get update --fix-missing
RUN apt-get install -y cmake g++ fltk1.1-dev libjpeg8-dev libpng-dev \
    libglu1-mesa-dev libltdl-dev gdb 

# install python depends
RUN pip3 install matplotlib 
RUN pip3 install scipy 
RUN pip3 install opencv-python
RUN apt install -y -qq python3-tk

# get pip and install catkin
WORKDIR "/workspaces/sst-misk-workspace"
RUN apt install -y python2.7
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
RUN python2.7 get-pip.py
RUN python2.7 -m pip install --upgrade pip
RUN pip install catkin_tools

RUN mkdir stage4 
WORKDIR "/workspaces/sst-misk-workspace/stage4"
RUN git clone git://github.com/rtv/Stage.git Stage
ENV STG=$HOME/stg
RUN cmake -DCMAKE_INSTALL_PREFIX=$STG Stage
RUN make
RUN make install
ENV LD_LIBRARY_PATH=$STG/lib
RUN echo 'export PATH=$STG/bin:$PATH' >> ~/.bashrc
# RUN echo 'source /workspaces/sst-misk-workspace/ros_ws/devel/setup.bash' >> ~/.bashrc
RUN echo '. /workspaces/sst-misk/ros_ws/devel/setup.bash' >> ~/.bashrc
# export path to controller used by stage while simulating the robot (looks like it's no longer needed, but for now stays just to be sure)
RUN echo 'export STAGEPATH=/workspaces/sst-misk-workspace/ros_ws/devel/lib' >> ~/.bashrc

WORKDIR "/workspaces"

CMD ["bash"]
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics