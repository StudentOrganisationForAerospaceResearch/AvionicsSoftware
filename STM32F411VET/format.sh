#!/bin/bash -e

astyle "Inc/*.h" "Src/*.c" -r --options=astylerc
find Inc/ -name "*.orig" -delete
find Src/ -name "*.orig" -delete
