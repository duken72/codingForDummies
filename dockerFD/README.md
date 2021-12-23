# Docker

- Short guide: [1](https://youtu.be/_dfLOzuIg2o).
- More comprehensive guide: [2](https://youtu.be/rOTqprHv1YE), [3](https://youtu.be/gAkwW2tuIqE).
- More sharings: [4](https://youtu.be/IbUXb4pQbPY), [5](https://youtu.be/u-YWtdbpEhQ).

-------

## Guide

CLI commands:

```bash
alias dockerg='less $(fd dockerFD.bash ~)'
alias dockergg='less $(fd dockerFD.bash ~) | grep'
```

Example use: search commands to install by tag:

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

## Using

Choose image(s) from [Docker hub](https://hub.docker.com/)
