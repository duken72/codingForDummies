###################################
# INSTALLATION
###################################
# Log in as root user
sudo pacman -S docker           #install pkg
sudo aura -A docker-git         #install pkg
# Start docker service
systemctl start docker.service  #install service
systemctl enable docker.service #install service
docker info                     #install check
# To run docker CLI commands as non-root user
usermod -aG docker [user_name]  #install non-root user
# Delete all Docker data (purge directory) #purge
rm -R /var/lib/docker           #purge


###################################
# IMAGES MANAGEMENT
###################################
# Example
docker pull ubuntu:16.04

# Pull image from Docker Hub
docker pull [image_name]        #image pull
docker pull [image_name:tag]    #image pull
docker tag [image_name]
docker push [image_name]        #image push
# List all images
docker images                   #image list
# Delete images by ID
docker rmi <IMAGE ID>           #image remove
docker system prune


###################################
# CONTAINERS MANAGEMENT
###################################
docker ps                   #container list
docker container ls         #container list
docker ps -a                #container - list all
docker stats                #container - CPU, mem usages 
# Stop, start, kill with ID or name
docker stop <CONTAINER ID>  #container stop
docker start <CONTAINER ID> #container start
docker kill <CONTAINER ID>  #container kill
docker rm <CONTAINER ID>    #container delete

# Build image from Dockerfile
docker build -t [image_name] [path]
docker build -t myDocker .

# Run container from an image
docker run [image_name]

# Enter the CLI within container
docker exec -it [containerID] bash
docker run -it ubuntu /bin/bash

# Using Docker Hub
docker pull image_name:tag
docker tag image_name:latest image_name:v1.0.0
docker push image_name
# Example
docker pull ubuntu
docker pull ubuntu:16.04
