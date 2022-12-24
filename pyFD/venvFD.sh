#!/bin/bash

python3 -m venv [dir]           # setup virtual env in dir
source /dir/bin/activate        # activate virtual env
deactivate                      # exit virtual env
rm -rf dir                      # remove the dir to rm the venv

pip list                        # list installed packages in venv
pip install pandas==1.1.1       # install pkg with specific version
pip install 'pandas>0.25.3'     # install pkg
pip freeze > requirements.txt   # save pkg list for reproduction
pip install -r requirements.txt # install given pkg list

pip install 'dm-acme[tf]'	# zsh can't escape square brackets, thus need ''
