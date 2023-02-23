# FROM tiryoh/ros-desktop-vnc
FROM ros:noetic

RUN apt-get update && \
	apt-get install -y \
	qt5-default \
	git && \
	mkdir -p /root/.local/share/applications