<!-- omit in toc -->
# Docker For Dummies

Docker is a tool to package software in a *"containers"* that can run reliably in any environment / with any hardware.\
It's a better replacement for traditional Virtual Machines (VM).

- An application should be compatible to many operating systems (OS) and types of machines (PC, laptop, smartphone, server)
- A Server is a piece of computer hardware (or/and software) that provides functionality for other programs or devices.   A server has CPU, RAM, hard drive, etc. It is basically just like your PC, but not at your home.
- Before VM, only one OS can be installed on a server (or any computer).\
  Thus, all resources of that server / computer would be dedicated to that OS (Windows, Linux, etc.)
- Then VM comes. You can now run multiple OS on one server machine with VM and a hypervisor. The hypervisor divides the server's resources to specified amount of resources.
- Prior problem with traditional VMs is that it just overloads everything. You can only run a limited number of VMs on a machine.
- VMs simulate hardware for other OS. While Docker simulates the OS directly, kind of. VMs will use different kernels, probably duplications of the same one.
- Docker communicates natively with the host system and uses less disk space.

<!-- omit in toc -->
## Table of Contents

- [Resources](#resources)
- [Basics](#basics)
- [CLI Commands](#cli-commands)
- [Install and Configure](#install-and-configure)
- [Guide](#guide)
- [Using given Dockerfile](#using-given-dockerfile)
- [Anatomy of a Dockerfile](#anatomy-of-a-dockerfile)
  - [Refs](#refs)
  - [Commands](#commands)
  - [Example](#example)
- [Docker with Python](#docker-with-python)

-------

## Resources

- [Docker docs](https://docs.docker.com/)
- Short guide: [TechSquidTV](https://youtu.be/_dfLOzuIg2o).
- More explanation guide: [Simplilearn](https://youtu.be/rOTqprHv1YE), [Fireship](https://youtu.be/gAkwW2tuIqE), [NetworkChuck](https://youtu.be/eGz9DS-aIeY).
- Sharing: [TechLead](https://youtu.be/IbUXb4pQbPY), [Telusko](https://youtu.be/u-YWtdbpEhQ).
- In-depth example: [Travis Media](https://youtu.be/i7ABlHngi1Q)

-------

## Basics

- A Dockerfile ⇾ an image ⇾ a container.
- Dockerfile is like the DNA or blueprint, it describes how the image will be built.
  - See example [Dockerfile](Dockerfile)
- The image is a snapshot of your software with all dependencies
- The container is where your actual software running.\
  The software, with everything (dependencies, memory, etc.) is separated with others and can be easily managed.

-------

## CLI Commands

```bash
alias dockerg='less $(fd dockerFD.bash ~)'
alias dockergg='less $(fd dockerFD.bash ~) | grep'
```

Example use: search commands to install by tag

```bash
dockergg install
```

Results:

```bash
sudo pacman -S docker #install pkg
sudo aura -A docker-git #install pkg
systemctl start docker.service #install service
systemctl enable docker.service #install service
docker info #install check
usermod -aG docker [user_name] #install non-root user
```

-------

## Install and Configure

Check [Arch Wiki](https://wiki.archlinux.org/title/docker) and `dockergg install` in CLI.

-------

## Guide

This is just a short guide for overall procedure. \
For details, check [dockerFD.bash](dockerFD.bash), [Docker docs](https://docs.docker.com/).

- Choose the base image/OS from [Docker hub](https://hub.docker.com/)

  ```bash
  docker pull ubuntu
  docker pull centos
  ```

- Manage images

  ```bash
  docker images # list images/OS
  docker image ... # manage images
  docker rmi <IMAGE ID> # remove image(s)
  ```

- Run the container

  ```bash
  docker run -dt --name testDocker ubuntu
  docker exec -it containerName bash
  cat /etc/os-release #within the container
  ```

- Manage container

  ```bash
  docker ps [-a]
  docker stop containerName #or containerID
  docker start containerName #or containerID
  docker kill containerName #or containerID
  docker rm containerName #or containerID
  docker container ... # manage container(s)
  ```

-------

## Using given Dockerfile

Given the Dockerfile:

- Build the image

  ```bash
  docker build -t [imageName] [pathTODockerfile]
  #Example: docker build -t test .
  docker images
  ```

- Run a container with the built image

  ```bash
  docker run -dt --name [containerName] [imageName:tag]
  docker ps -a
  ```

-------

## Anatomy of a Dockerfile

### Refs

- [adamveld12](https://gist.github.com/adamveld12/4815792fadf119ef41bd)
- [Justen Mehl](https://mehlj.github.io/Dockerfile/)
- [Mehmet Barış Kalkar](https://dev.to/mbaris/anatomy-of-a-dockerfile-4b4p)
- [Docs Docker](https://docs.docker.com/engine/reference/builder/)
- [Dockerfile Best practices](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/)
- [Dockerfile: `ENTRYPOINT` vs `CMD`](https://www.ctl.io/developers/blog/post/dockerfile-entrypoint-vs-cmd/)

### Commands

- `FROM`: set base image, usually the OS, but not necessary always is an OS

  ```docker
  FROM ubuntu # <image>
  FROM ubuntu:latest # - <image>:<tag>
  FROM ubuntu:precise (LTS)
  ```

- `LABEL`: add metadata (maintainer, version, etc)

  ```docker
  LABEL maintainer="Duck"
  LABEL version="1.1"
  LABEL org.opencontainers.image.authors="duck@gmail.com"
  ```

- `COPY` & `ADD`: copying of local files into the container.\
  The best use for ADD is local `.tar` file auto-extraction into the image.\
  Using `ADD` to fetch packages from remote `URLs` is strongly discouraged; use `curl` or `wget` instead.

  ```docker
  COPY requirements.txt /tmp/
  RUN pip install --requirement /tmp/requirements.txt
  COPY . /tmp/
  ADD rootfs.tar.xz /
  ```

- `ENV`: set env variables

  ```docker
  ENV KEY 1234 # <key> <value>
  ```

- `WORKDIR`: sets the working directory to run instructions: `CMD`, `RUN`, `ENTRYPOINT` and `COPY` after this step.
  
  ```docker
  WORKDIR /project
  ```

- `RUN` & `CMD` & `ENTRYPOINT`: Run commands
  - `RUN`: Runs a command on the image and commits the result
  - `CMD`:
  - `ENTRYPOINT`:
  
  ```docker
  RUN apt-get install vim
  CMD ["node", "src/index.js"]
  ENTRYPOINT ["wc", "-l", "-"]
  ```

- `EXPOSE`: exposes port on the container to the outside world
  
  ```docker
  EXPOSE 80 # <port> [<port>...]
  ```

- `VOLUME`: adds more new volume(s) to any container created from the image

  ```docker
  VOLUME ["/data"] # [<volumes>...], puts /data -> /var/lib/docker/volumes/
  ```

### Example

Check out the [Dockerfile](dockerTest/Dockerfile)

```bash
cd dockerTest
docker build -t testdockerfile .
```

The line `RUN while true; do sleep 1000; done` is just to keep the container running for examination.

You can enter the container with VSCode Docker extension or in CLI

```bash
docker ps
docker exec -it [containerName] bash
```

You will see:

- Current directory is `"/home"`
- File copied to `"/tmp"`
- `test.tar.gz` extracted at `"/"` and `"/home"`
- New volume `"/home/test_volume"`

-------

## Docker with Python

Check [dockerPy](dockerPy/Dockerfile)

Resources: Dockerize a Python application

- [runnable.com](https://runnable.com/docker/python/dockerize-your-python-application)
- [Docker Docs](https://docs.docker.com/language/python/build-images/)
