# Dependencies

- C++ compiler > C++ 17
- Boost > 1.70
- Python > 3.7
- GPU
- Cuda > 11.0


# Building

'''
cmake -D TBB_INCLUDEDIR=$TBB_DIR/include -D TBB_LIBRARYDIR=$TBB_DIR/lib/intel64/gcc4.8/ -D CMAKE_BUILD_TYPE=Release ..
make
'''