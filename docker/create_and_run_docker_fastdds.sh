if [ -n $1 ]; then
    NAME=$1
else
    NAME=rosgalactic
fi

sudo docker run -it --privileged --net=bridge\
    --env="DISPLAY"\
    --env="QT_X11_NO_MITSHM=1"\
    --name=$NAME\
    --volume="/dev":"/dev"\
    --volume="/tmp/.X11-unix":"/tmp/.X11-unix":rw\
    --volume="/home/victor/work/cloud-robotics/work":"/root/work":rw\
    galactic_fastdds bash
