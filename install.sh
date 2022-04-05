NJOBS=${1:-4}

echo "Building workspace using $NJOBS cores"

# Environment variables
export DIR_PROJECT_ROOT=$(pwd)
export LIB_DIR=$DIR_PROJECT_ROOT/lib
export PACKAGE_DIR=$DIR_PROJECT_ROOT/packages


echo "Creating Directories"
mkdir -p $LIB_DIR

echo "Installing dependencies..."

## Dependencies

# json
echo "Installing JSON for C++"
tar -xf $PACKAGE_DIR/json.tar.xz -C $LIB_DIR

# plog
echo "Installing plog"
tar -xf $PACKAGE_DIR/plog-1.1.6.tar.gz -C $LIB_DIR

# TBB
echo "Installing TBB"
tar -xf $PACKAGE_DIR/oneapi-tbb-2021.5.0-lin.tgz -C $LIB_DIR
export TBB_DIR=$LIB_DIR/oneapi-tbb-2021.5.0

# BLOSC
echo "Installing C-BLOSC"
tar -xf $PACKAGE_DIR/c-blosc-1.21.1.tar.gz -C $LIB_DIR
export BLOSC_DIR=$LIB_DIR/c-blosc-1.21.1
export BLOSC_BUILD_DIR=$BLOSC_DIR/build

echo "Building C-BLOSC"
cmake \
    -D CMAKE_INSTALL_PREFIX=$BLOSC_BUILD_DIR \
    -D CMAKE_BUILD_TYPE=Release \
    -S $BLOSC_DIR \
    -B $BLOSC_BUILD_DIR 
cmake --build $BLOSC_BUILD_DIR -j$NJOBS
cmake --install $BLOSC_BUILD_DIR 

# OpenVDB
echo "Installing OpenVDB"
tar -xf $PACKAGE_DIR/openvdb-9.0.0_modified.tar.gz -C $LIB_DIR

export OPENVDB_DIR=$LIB_DIR/openvdb-9.0.0_modified
export OPENVDB_BUILD_DIR=$OPENVDB_DIR/build
export OPENVDB_INSTALL_DIR=$OPENVDB_DIR


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
    -D OPENVDB_FUTURE_DEPRECATION=OFF \
    -D TBB_INCLUDEDIR=$TBB_DIR/include \
    -D TBB_LIBRARYDIR=$TBB_DIR/lib/intel64/gcc4.8 \
    -D BLOSC_INCLUDEDIR=$BLOSC_BUILD_DIR/include \
    -D BLOSC_LIBRARYDIR=$BLOSC_BUILD_DIR/lib \
    -D CMAKE_INSTALL_PREFIX=$OPENVDB_INSTALL_DIR \
    -D CMAKE_INSTALL_LIBDIR=lib \
    -B $OPENVDB_BUILD_DIR \
    -S $OPENVDB_DIR
make -C $OPENVDB_BUILD_DIR -j$NJOBS
make -C $OPENVDB_BUILD_DIR install


# # Python virtual environment
# echo "Setting up Python virtual environment"
# python3 -m venv .venv
# source .venv/bin/activate
# python3 -m pip install -r PyRequirements.txt