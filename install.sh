# TODO: install OVDB in dir that does not require root

# Environment variables
export DIR_PROJECT_ROOT=$(pwd)
export DIR_BUILD=$DIR_PROJECT_ROOT/build

export DIR_OPENVDB=$DIR_PROJECT_ROOT/openvdb
export DIR_OPENVDB_BUILD=$DIR_OPENVDB/build
export DIR_BOOST=$DIR_PROJECT_ROOT/boost



# install dependencies
sudo apt-get install -y \
    libboost-iostreams-dev \
    libtbb-dev \
    libblosc-dev \
    libpthread-stubs0-dev \
    libgtest-dev \
    doxygen \
    libgl1-mesa-dev \
    libglfw3-dev \
    libopengl-dev \
    libjemalloc-dev \ 
    freeglut3-dev
    
# Tools
sudo apt install paraview

# build openvdb
git clone https://github.com/AcademySoftwareFoundation/openvdb
git pull
mkdir -p $DIR_OPENVDB_BUILD
( \
    cd $DIR_OPENVDB_BUILD \
    && cmake \
    -D OPENVDB_BUILD_VDB_PRINT=ON \
    -D OPENVDB_BUILD_VDB_LOD=ON \
    -D OPENVDB_BUILD_VDB_RENDER=ON \
    -D OPENVDB_BUILD_VDB_VIEW=ON \
    -D OPENVDB_BUILD_UNITTESTS=ON \
    -D OPENVDB_BUILD_VDB_RENDER=ON \
    .. \
    && make -j4 \
    && sudo make install \
)
#make -j8 openvdb
#sudo make install
