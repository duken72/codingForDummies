# Installation
# Log in as root user
sudo pacman -S docker #install pkg
sudo aura -A docker-git #install pkg
# Start docker service
systemctl start docker.service #install service
systemctl enable docker.service #install service
docker info #install check
# To run docker CLI commands as non-root user
usermod -aG docker [user_name] #install non-root user
# Delete all Docker data (purge directory) #purge
rm -R /var/lib/docker #purge

# Images Managing
docker pull [image_name] #images pull
# check running containers
docker ps #container
# list all containers running on host for deletion
docker ps -a #container
docker stop <CONTAINER ID> #container stop running
docker kill <CONTAINER ID> #container kill running
docker rm <CONTAINER ID> #container delete

docker images # List all Docker images
# Delete images by ID
docker rmi <IMAGE ID> #images
docker system prune

# Usage
docker run -it ubuntu /bin/bash #usage

# Using Docker Hub
docker pull image_name
docker tag image_name
docker push image_name