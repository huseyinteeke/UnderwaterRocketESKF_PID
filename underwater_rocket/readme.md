cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=arm-gcc-toolchain.cmake
cmake --build build -j4