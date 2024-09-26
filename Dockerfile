FROM ros:humble

RUN apt update
RUN apt install ros-humble-pcl-ros -y
