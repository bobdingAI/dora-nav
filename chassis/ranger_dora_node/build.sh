#!/bin/bash

# Build script for Ranger DORA Node

echo "========================================="
echo "Building Ranger Mini V3 DORA Node"
echo "========================================="

# Create build directory
mkdir -p build
cd build

# Run CMake
echo "Running CMake..."
cmake ..

# Build
echo "Building..."
make -j$(nproc)

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "========================================="
    echo "Build successful!"
    echo "Executable: build/ranger_miniv3_node"
    echo "========================================="
else
    echo "========================================="
    echo "Build failed!"
    echo "========================================="
    exit 1
fi
