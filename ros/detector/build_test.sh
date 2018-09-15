export LD_LIBRARY_PATH = "${LD_LIBRARY_PATH}:$(pwd)/darknet"
g++ -std=c++11  -DOPENCV `pkg-config --cflags opencv` -Wall -Ofast -DOPENCV -fopenmp -fPIC -o go src/yolo.cpp -lm -pthread  `pkg-config --libs opencv`  -lgomp -L ./ -l:darknet/darknet.so
