# TODO: install OVDB in dir that does not require root

# Environment variables
export DIR_PROJECT_ROOT=$(pwd)
export DIR_OPENVDB=$DIR_PROJECT_ROOT/lib/openvdb
export DIR_OPENVDB_BUILD=$DIR_OPENVDB/build

# # install dependencies
# sudo apt-get install \
#     pkg-config \
#     libboost-all-dev \
#     build-essential \
#     libgl1-mesa-dev \
#     freeglut3-dev \
#     libboost-iostreams-dev \
#     libtbb-dev \
#     libblosc-dev \
#     libpthread-stubs0-dev \
#     nvidia-cuda-dev \
#     libboost-program-options-dev
#     nlohmann-json3-dev \
#     libjemalloc-dev
    
# Tools
# sudo apt install paraview
sudo apt install python3 python3-pip
pip3 install pandas numpy


# build openvdb
git -C  $DIR_OPENVDB  pull || git clone https://github.com/AcademySoftwareFoundation/openvdb $DIR_OPENVDB 
mkdir -p $DIR_OPENVDB_BUILD
( \
    cd $DIR_OPENVDB_BUILD \
    && cmake \
    -D OPENVDB_BUILD_VDB_PRINT=OFF \
    -D OPENVDB_BUILD_VDB_LOD=OFF \
    -D OPENVDB_BUILD_VDB_RENDER=OFF \
    -D OPENVDB_BUILD_VDB_VIEW=OFF \
    -D OPENVDB_BUILD_UNITTESTS=OFF \
    -D OPENVDB_BUILD_VDB_RENDER=OFF \
    -D OPENVDB_BUILD_NANOVDB=ON \
    -D CMAKE_INSTALL_PREFIX=$DIR_OPENVDB
    .. \
    && make -j4 \
    && sudo make install \
)
#make -j8 openvdb
#sudo make install
