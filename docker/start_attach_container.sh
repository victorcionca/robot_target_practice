if [ -n "$1" ]; then
    NAME=$1
else
    echo "Usage: sh start_attach_container CONTAINER_NAME"
    exit
fi

xhost +
sudo docker start $NAME
sudo docker attach $NAME
xhost -
