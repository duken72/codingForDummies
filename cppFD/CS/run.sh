#!/bin/bash

clear

g++ -std=c++20 -DDEBUG=false -Wall "$1" -o out

INPUT="${1%'.cpp'}.txt"
if [ -f $INPUT ]; then
    ./out < $INPUT
else
    ./out
fi
rm -f out
