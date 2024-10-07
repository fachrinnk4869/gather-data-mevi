xhost +local:docker
sudo docker compose up -d
sudo docker exec -it zed_jetson_camera_2 /bin/bash