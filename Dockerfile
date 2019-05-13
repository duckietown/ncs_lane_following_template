FROM andyser/ncs_lane_following_template:ver_1

LABEL maintainer="andyser.eed04@nctu.edu.tw"

# Enable QEMU for ARM to build ARM image on X86 machine
COPY ./qemu-arm-static /usr/bin/qemu-arm-static

COPY ncs_following /home/software/catkin_ws/src/

RUN /bin/bash -c "cd /home/software/ && source environment.sh && cd catkin_ws && catkin_make --pkg ncs_following"

WORKDIR /home/software/
COPY run_ncslanefollowingdemo.sh .

ENTRYPOINT /bin/bash run_ncslanefollowingdemo.sh
