#!/bin/bash

COLOR_RED='\033[1;31m'
NO_COLOR='\033[0m'
COLOR_BLUE='\033[0;34m'

DOC_URL=https://google.com
echo "Opening '$DOC_URL' for you .."

echo -e "Opening ${COLOR_RED}'$DOC_URL' for you ..${NO_COLOR}"
echo -e "Normal color ${COLOR_RED}red ${COLOR_BLUE}blue${NO_COLOR} normal"
echo
sensible-browser $DOC_URL
