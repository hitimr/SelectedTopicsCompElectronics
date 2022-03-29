# TODO: install OVDB in dir that does not require root

# Environment variables
export DIR_PROJECT_ROOT=$(pwd)


# # install dependencies
# sudo apt-get install \
#     build-essential \
#     # freeglut3-dev \
#     libboost-iostreams-dev \
#     libtbb-dev \
#     libblosc-dev \
#     libpthread-stubs0-dev \
#     nvidia-cuda-dev \
#     libboost-program-options-dev
#     nlohmann-json3-dev \
#     libjemalloc-dev
    
# # Tools
# sudo apt install python3 python3-pip
# pip3 install pandas numpy matplotlib


## Dependencies
# TBB
export TBB_DIR=$DIR_PROJECT_ROOT/lib/oneTBB
export TBB_BUILD_DIR=$TBB_DIR/build
git -C  $TBB_DIR  pull || git clone https://github.com/oneapi-src/oneTBB $TBB_DIR 
mkdir -p $TBB_BUILD_DIR 

cmake \
    -D CMAKE_INSTALL_PREFIX=$TBB_BUILD_DIR \
    -D TBB_TEST=OFF \
    -S $TBB_DIR \
    -B $TBB_BUILD_DIR
cmake --build $TBB_BUILD_DIR --config Release -j4 
cmake --install $TBB_BUILD_DIR 


# BLOSC
export BLOSC_DIR=$DIR_PROJECT_ROOT/lib/c-blosc
export BLOSC_BUILD_DIR=$BLOSC_DIR/build
git -C  $BLOSC_DIR  pull || git clone https://github.com/oneapi-src/oneTBB $BLOSC_DIR 
mkdir -p $BLOSC_BUILD_DIR 

cmake \
    -D CMAKE_INSTALL_PREFIX=$BLOSC_BUILD_DIR \
    -S $BLOSC_DIR \
    -B $BLOSC_BUILD_DIR 
cmake --build $BLOSC_BUILD_DIR -j4
cmake --install $BLOSC_BUILD_DIR 


# OpenVDB
export DIR_OPENVDB=$DIR_PROJECT_ROOT/lib/openvdb
export DIR_OPENVDB_BUILD=$DIR_OPENVDB/build
export OPENVDB_INSTALL_DIR=$HOME/lib/openvdb
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
    -D TBB_INCLUDEDIR=$TBB_BUILD_DIR/include \
    -D TBB_LIBRARYDIR=$TBB_BUILD_DIR/lib64 \
    -D BLOSC_INCLUDEDIR=$BLOSC_BUILD_DIR/include \
    -D BLOSC_LIBRARYDIR=$BLOSC_BUILD_DIR/lib64 \
    -D CMAKE_PREFIX_PATH=$DIR_OPENVDB \
    -D CMAKE_INSTALL_PREFIX=$OPENVDB_INSTALL_DIR \
    -B $DIR_OPENVDB_BUILD \
    -S $DIR_OPENVDB
cmake --build $DIR_OPENVDB_BUILD -j4
cmake --install $DIR_OPENVDB_BUILD 
