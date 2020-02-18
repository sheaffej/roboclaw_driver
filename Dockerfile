FROM ros:melodic-ros-base

ENV ROS_WS /ros
ENV PYTEST_ADDOPTS "--color=yes"
SHELL [ "bash", "-c"]

# Install pip 
RUN apt update \
	&& apt install -y \
		libffi-dev \
		libssl-dev \
		python-pip \
	&& pip install --no-cache-dir --upgrade pip \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Install Python testing packages
RUN pip install \
		pytest \
		pytest-cov \
		coveralls

RUN pip install \
	pyserial

COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" ]

RUN echo "source /entrypoint.sh" >> /root/.bashrc && \
	echo "source /root/.bashrc" >> /root/.bash_profile

RUN mkdir -p ${ROS_WS}/src

COPY . ${ROS_WS}/src/roboclaw_driver/

RUN cd ${ROS_WS} \
&& apt update \
&& rosdep update \
&& rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && \
	cd $ROS_WS && catkin_make && \
	rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /ros/src/roboclaw_driver
