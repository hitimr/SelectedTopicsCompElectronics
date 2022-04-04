NJOBS=${1:-4}

echo "Building workspace using $NJOBS cores"

# Environment variables
export DIR_PROJECT_ROOT=$(pwd)
export DOWNLOAD_DIR=$LIB_DIR/download
export LIB_DIR=$DIR_PROJECT_ROOT/lib

echo "Creating Directories"

# Python virtual environment
echo "Setting up Python virtual environment"
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r PyRequirements.txt

## Dependencies

# json
tar -xf $DOWNLOAD_DIR/json.tar.xz -C $DIR_PROJECT_ROOT/lib/download
 

# plog
echo "Downloading plog"
export PLOG_DIR=$DIR_PROJECT_ROOT/lib/plog
git -C  $PLOG_DIR  pull || git clone https://github.com/SergiusTheBest/plog $PLOG_DIR --depth 1

# TBB
echo "Installing TBB"
export TBB_SOURCE_DIR=$DIR_PROJECT_ROOT/lib/download/oneTBB
export TBB_BUILD_DIR=$TBB_SOURCE_DIR/build
export TBB_INSTALL_DIR=$DIR_PROJECT_ROOT/lib/oneTBB

git -C  $TBB_SOURCE_DIR  pull || git clone https://github.com/oneapi-src/oneTBB $TBB_SOURCE_DIR --depth 1
mkdir -p $TBB_BUILD_DIR 
mkdir -p $TBB_INSTALL_DIR 

cmake \
    -D CMAKE_INSTALL_PREFIX=$TBB_INSTALL_DIR \
    -D TBB_TEST=OFF \
    -S $TBB_SOURCE_DIR \
    -B $TBB_BUILD_DIR
cd $TBB_BUILD_DIR
cmake --build $TBB_BUILD_DIR -j$NJOBS ..
make install 

cd $DIR_PROJECT_ROOT

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
export OPENVDB_SOURCE_DIR=$DIR_PROJECT_ROOT/lib/download/openvdb-9.0.0
export OPENVDB_BUILD_DIR=$OPENVDB_SOURCE_DIR/lib/openvdb/build
export OPENVDB_INSTALL_DIR=$DIR_PROJECT_ROOT/openVDB

wget -nc https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v9.0.0.tar.gz -P $DIR_PROJECT_ROOT/lib/download
tar -xf $DIR_PROJECT_ROOT/lib/download/v9.0.0.tar.gz -C $DIR_PROJECT_ROOT/lib/download
mkdir -p $OPENVDB_BUILD_DIR

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
    -D OPENVDB_USE_DEPRECATED_ABI=ON \
    -D OPENVDB_USE_DEPRECATED_ABI_6=ON \
    -D OPENVDB_USE_DEPRECATED_ABI_7=ON \
    -D OPENVDB_FUTURE_DEPRECATION=OFF \
    -D TBB_INCLUDEDIR=$TBB_INSTALL_DIR/include \
    -D TBB_LIBRARYDIR=$TBB_INSTALL_DIR/lib \
    -D BLOSC_ROOT=$BLOSC_BUILD_DIR \
    -D BLOSC_INCLUDEDIR=$BLOSC_BUILD_DIR/include \
    -D BLOSC_LIBRARYDIR=$BLOSC_BUILD_DIR/lib64 \
    -D CMAKE_INSTALL_PREFIX=$OPENVDB_INSTALL_DIR \
    -D CMAKE_INSTALL_LIBDIR=lib64 \
    -B $OPENVDB_BUILD_DIR \
    -S $OPENVDB_SOURCE_DIR
make -C $OPENVDB_BUILD_DIR -j$NJOBS
make -C $OPENVDB_BUILD_DIR install
