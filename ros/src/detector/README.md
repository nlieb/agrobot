# Setup

Clone:
```bash
git clone https://github.com/AlexeyAB/darknet.git
cd darknet
# Change Makefile depending on platform
```

Makefile example:

GPU=1
CUDNN=1
CUDNN_HALF=0
OPENCV=1
AVX=1
OPENMP=1
LIBSO=1


Ask Nico for a weights file and copy it in this directory.

./build_test.sh
./go sample/00186.jpg