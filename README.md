# yolo-sort

## install
1. Eigen
apt 
```shell
sudo apt install libeigen3-dev
```
source code
```shell
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build
cd build
cmake ..
sudo make install
# make blas
```

## example
### test_kalman.cpp
simple example of kalman filter.