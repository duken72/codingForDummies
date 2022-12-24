###################################
# INSTALLATION
###################################
# Log in as root user
sudo pacman -S docker           # install pkg
sudo aura -A docker-git         # install pkg
# Start docker service
systemctl start docker.service  # install service
systemctl enable docker.service # install service
docker info                     # install check
# To run docker CLI commands as non-root user
usermod -aG docker [user_name]  # install non-root user
# Delete all Docker data (purge directory) #purge
rm -R /var/lib/docker           # purge


###################################
# IMAGES MANAGEMENT
###################################
# Example
docker pull ubuntu:16.04

# Pull image from Docker Hub
docker pull [image_name]        # image pull
docker pull [image_name:tag]    # image pull
docker tag [image_name]         # image tag
docker push [image_name]        # image push
docker images                   # image list all images
docker rmi <IMAGE ID>           # image remove by ID
docker system prune


###################################
# CONTAINERS MANAGEMENT
###################################
docker ps                   # container list
docker container ls         # container list
docker ps -a                # container - list all
docker stats                # container - CPU, mem usages 
docker stop <CONTAINER ID>  # container stop with ID/name
docker start <CONTAINER ID> # container start with ID/name
docker kill <CONTAINER ID>  # container kill with ID/name
docker rm <CONTAINER ID>    # container delete with ID/name

docker build -t [image_name] [path] # build image from Dockerfile
docker build -t myDocker .          # build image from Dockerfile

docker run [image_name]     # Run container from an image

docker exec -it [containerID] bash  # Enter the CLI within container
docker run -it ubuntu /bin/bash     # Enter the CLI within container

# Using Docker Hub
docker pull image_name:tag                      # Docker Hub
docker tag image_name:latest image_name:v1.0.0  # Docker Hub
docker push image_name                          # Docker Hub
# Example
docker pull ubuntu
docker pull ubuntu:16.04
