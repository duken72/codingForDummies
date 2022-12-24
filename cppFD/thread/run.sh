#!/bin/bash

clear

g++ -std=c++0x -Wall -lpthread thread-function.cpp  -o thread-function
g++ -std=c++0x -Wall -lpthread thread-lambda.cpp    -o thread-lambda
g++ -std=c++0x -Wall -lpthread thread-functor.cpp   -o thread-functor

# ./thread-function
# ./thread-lambda
# ./thread-functor

hyperfine ./thread-function ./thread-lambda ./thread-functor
rm -f thread-lambda thread-functor thread-function
