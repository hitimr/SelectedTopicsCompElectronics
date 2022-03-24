# TODO: install OVDB in dir that does not require root

# Environment variables
export DIR_PROJECT_ROOT=$(pwd)
export DIR_OPENVDB=$DIR_PROJECT_ROOT/lib/openvdb
export DIR_OPENVDB_BUILD=$DIR_OPENVDB/build
export OPENVDB_INSTALL_DIR=$HOME/lib/openvdb

# install dependencies
sudo apt-get install \
    build-essential \
    # freeglut3-dev \
    libboost-iostreams-dev \
    libtbb-dev \
    libblosc-dev \
    libpthread-stubs0-dev \
    nvidia-cuda-dev \
    libboost-program-options-dev
    nlohmann-json3-dev \
    libjemalloc-dev
    
# Tools
sudo apt install python3 python3-pip
pip3 install pandas numpy matplotlib

# build openvdb
git -C  $DIR_OPENVDB  pull || git clone https://github.com/AcademySoftwareFoundation/openvdb $DIR_OPENVDB 
mkdir -p $DIR_OPENVDB_BUILD
cmake \
    -D OPENVDB_BUILD_VDB_PRINT=OFF \
    -D OPENVDB_BUILD_VDB_LOD=OFF \
    -D OPENVDB_BUILD_VDB_RENDER=OFF \
    -D OPENVDB_BUILD_VDB_VIEW=OFF \
    -D OPENVDB_BUILD_UNITTESTS=OFF \
    -D OPENVDB_BUILD_VDB_RENDER=OFF \
    -D OPENVDB_BUILD_NANOVDB=ON \
    -D CMAKE_PREFIX_PATH=$DIR_OPENVDB \
    -B $DIR_OPENVDB_BUILD \
    -S $DIR_OPENVDB \
    -D CMAKE_INSTALL_PREFIX=$OPENVDB_INSTALL_DIR

make -C $DIR_OPENVDB_BUILD 
