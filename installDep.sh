#!/bin/bash
cd Thirdparty/eigen-3.1.3
mkdir build
cd build
cmake ..
sudo make install
cd ../../../
echo "Configuring and building Thirdparty/DBoW2 ..."
#sudo apt install libeigen3-dev
cd Thirdparty/DBoW2
mkdir build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DEigen3_INCLUDE_DIR=/usr/local/include/eigen3
make -j$(nproc)

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DEigen3_INCLUDE_DIR=/usr/local/include/eigen3
make -j$(nproc)
echo "Configuring and building Thirdparty/Pangolin ..."
sudo apt-get install libglew-dev
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt-get install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev
sudo apt-get install libdc1394-22-dev libraw1394-dev
sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev
sudo apt-get install libboost-all-dev libopenblas-dev

cd ../../Pangolin
mkdir build
cd build
cmake -DCPP11_NO_BOOST=1 -DEigen3_INCLUDE_DIR=/usr/local/include/eigen3 ..
sudo make -j$(nproc) install

cd ~
git clone https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
cmake -DEigen3_INCLUDE_DIR=/usr/local/include/eigen3 ..
sudo make -j$(nproc) install


