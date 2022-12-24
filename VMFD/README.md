<!-- omit in toc -->
# Virtual Machines for dummies

Some notes on Virtual Machines and the tool VirtualBox.

<!-- omit in toc -->
## Table of Contents

- [Sources](#sources)
- [Running VM from CLI](#running-vm-from-cli)

-------

## Sources

- [How to install and run Ubuntu on VM](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#3-install-your-image)

-------

## Running VM from CLI

Source:

- [techrepublic.com](https://www.techrepublic.com/article/how-to-run-virtualbox-virtual-machines-from-the-command-line/)

```bash
VBoxManage list vms
VBoxManage startvm "Ubuntu Server" --type headless
VBoxManage controlvm "Ubuntu Server" pause --type headless
VBoxManage controlvm "Ubuntu Server" resume --type headless
VBoxManage controlvm "Ubuntu Server" poweroff --type headless
```
