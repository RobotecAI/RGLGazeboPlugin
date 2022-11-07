cd ..
rm -rf build
mkdir build
cd build
cmake ..
make -j
cd ..
cd test_world