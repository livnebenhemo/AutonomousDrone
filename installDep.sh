echo "Configuring and building Thirdparty/DBoW2 ..."
sudo apt install libeigen3-dev
cd Thirdparty/DBoW2
mkdir build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

