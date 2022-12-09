#!/bin/bash
make clean
make all
export LD_LIBRARY_PATH=${PWD}
./iterative-walk 1000000 1 >> iterative-walk-100000-1.txt
./iterative-walk 1000000 2 >> iterative-walk-100000-2.txt
unset LD_LIBRARY_PATH