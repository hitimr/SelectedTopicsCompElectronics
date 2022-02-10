# install dependencies
sudo apt-get install -y libboost-iostreams-dev
sudo apt-get install -y libtbb-dev
sudo apt-get install -y libblosc-dev
sudo apt-get install -y libpthread-stubs0-dev
sudo apt-get install -y libgtest-dev

# build openvdb
git clone https://github.com/AcademySoftwareFoundation/openvdb
cd openvdb
mkdir build
cd build
cmake ..
make -j4
sudo make install