NJOBS=${1:-4}

echo "Building workspace using $NJOBS cores"

# Environment variables
export DIR_PROJECT_ROOT=$(pwd)

# Python virtual environment
echo "Setting up Python virtual environment"
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r PyRequirements.txt



## Dependencies
mkdir -p lib

# json
echo "Downloading json for C++"
export JSON_DIR=$DIR_PROJECT_ROOT/lib/json
git -C  $JSON_DIR  pull || git clone https://github.com/nlohmann/json $JSON_DIR --depth 1
 

# plog
echo "Downloading plog"
export PLOG_DIR=$DIR_PROJECT_ROOT/lib/plog
git -C  $PLOG_DIR  pull || git clone https://github.com/SergiusTheBest/plog $PLOG_DIR --depth 1

# TBB
echo "Installing TBB"
export TBB_DIR=$DIR_PROJECT_ROOT/lib/oneTBB
export TBB_BUILD_DIR=$TBB_DIR/build
git -C  $TBB_DIR  pull || git clone https://github.com/oneapi-src/oneTBB $TBB_DIR --depth 1
mkdir -p $TBB_BUILD_DIR 

cmake \
    -D CMAKE_INSTALL_PREFIX=$TBB_BUILD_DIR \
    -D TBB_TEST=OFF \
    -D CMAKE_INSTALL_LIBDIR=lib64 \
    -D CMAKE_BUILD_TYPE=Release \
    -S $TBB_DIR \
    -B $TBB_BUILD_DIR
cmake --build $TBB_BUILD_DIR --config Release -j$NJOBS 
cmake --install $TBB_BUILD_DIR 


# BLOSC
echo "Installing BLOSC"
export BLOSC_DIR=$DIR_PROJECT_ROOT/lib/c-blosc
export BLOSC_BUILD_DIR=$BLOSC_DIR/build
git -C  $BLOSC_DIR  pull || git clone https://github.com/oneapi-src/oneTBB $BLOSC_DIR --depth 1
mkdir -p $BLOSC_BUILD_DIR 

cmake \
    -D CMAKE_INSTALL_PREFIX=$BLOSC_BUILD_DIR \
    -D CMAKE_BUILD_TYPE=Release \
    -S $BLOSC_DIR \
    -B $BLOSC_BUILD_DIR 
cmake --build $BLOSC_BUILD_DIR -j$NJOBS
cmake --install $BLOSC_BUILD_DIR 

# OpenVDB
echo "Installing OpenVDB"
export DIR_OPENVDB=$DIR_PROJECT_ROOT/lib/openvdb
export DIR_OPENVDB_BUILD=$DIR_OPENVDB/build
git -C  $DIR_OPENVDB  pull || git clone https://github.com/AcademySoftwareFoundation/openvdb $DIR_OPENVDB --depth 1
mkdir -p $DIR_OPENVDB_BUILD

cmake \
    -D OPENVDB_BUILD_CORE=ON \
    -D OPENVDB_BUILD_BINARIES=ON \
    -D OPENVDB_BUILD_VDB_PRINT=OFF \
    -D OPENVDB_BUILD_VDB_LOD=OFF \
    -D OPENVDB_BUILD_VDB_RENDER=OFF \
    -D OPENVDB_BUILD_VDB_VIEW=OFF \
    -D OPENVDB_BUILD_UNITTESTS=OFF \
    -D OPENVDB_BUILD_VDB_RENDER=OFF \
    -D OPENVDB_BUILD_NANOVDB=ON \
    -D OPENVDB_INSTALL_CMAKE_MODULES=ON \
    -D TBB_ROOT=$TBB_BUILD_DIR \
    -D TBB_INCLUDEDIR=$TBB_BUILD_DIR/include \
    -D TBB_LIBRARYDIR=$TBB_BUILD_DIR/lib64 \
    -D BLOSC_ROOT=$BLOSC_BUILD_DIR \
    -D BLOSC_INCLUDEDIR=$BLOSC_BUILD_DIR/include \
    -D BLOSC_LIBRARYDIR=$BLOSC_BUILD_DIR/lib64 \
    -D CMAKE_PREFIX_PATH=$DIR_OPENVDB \
    -D CMAKE_INSTALL_PREFIX=$DIR_OPENVDB_BUILD \
    -D CMAKE_INSTALL_LIBDIR=lib64 \
    -D CMAKE_BUILD_TYPE=Release \
    -B $DIR_OPENVDB_BUILD \
    -S $DIR_OPENVDB
make -C $DIR_OPENVDB_BUILD -j$NJOBS
make -C $DIR_OPENVDB_BUILD install
