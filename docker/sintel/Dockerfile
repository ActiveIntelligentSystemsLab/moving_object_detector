FROM ros:melodic-perception

RUN apt-get update && \
    apt-get install -y wget unzip python-catkin-tools vim byobu

# Download and extract Sintel Dataset
WORKDIR /root
RUN wget http://files.is.tue.mpg.de/jwulff/sintel/MPI-Sintel-depth-training-20150305.zip
RUN wget http://files.is.tue.mpg.de/jwulff/sintel/MPI-Sintel-stereo-training-20150305.zip
ENV SINTEL_DEPTH_DIR=/root/MPI-Sintel-depth-training
RUN mkdir ${SINTEL_DEPTH_DIR} && \
    unzip MPI-Sintel-depth-training-20150305.zip -d ${SINTEL_DEPTH_DIR}
ENV SINTEL_STEREO_DIR=/root/MPI-Sintel-stereo-training
RUN mkdir ${SINTEL_STEREO_DIR} && \
    unzip MPI-Sintel-stereo-training-20150305.zip -d ${SINTEL_STEREO_DIR}

# Build
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN git clone https://github.com/ActiveIntelligentSystemsLab/sintel_stereo_publisher src/sintel_stereo_publisher
#RUN bash -c "source /opt/ros/melodic/setup.bash && \
#             rosdep install -i -y -r --from-paths src"
RUN bash -c "source /opt/ros/melodic/setup.bash && \
             catkin init && \
             catkin build"

# Replace entrypoint to load catkin_ws
RUN rm /ros_entrypoint.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Load ROS environment at docker exec bash
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
