FROM robustify101/ros:melodic-igvc-ci
MAINTAINER mradov@gmail.com

RUN apt update

COPY . /home/ros/src
RUN apt update
RUN ./ros_entrypoint.sh && cd /home/ros && rosdep install --from-paths src --ignore-src -y

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
