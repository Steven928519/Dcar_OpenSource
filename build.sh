#!/bin/bash

# STM32 Intelligent Build Script for Mac
# Usage: 
#   ./build.sh        - Just build code
#   ./build.sh flash  - Build and flash to board
#   ./build.sh clean  - Clean build directory

TARGET_BIN="build/DCar_OpenSource.bin"
START_ADDRESS="0x08000000"

echo "🚀 Starting build process..."

# Build with multiple cores
make -j$(sysctl -n hw.ncpu)

if [ $? -eq 0 ]; then
    echo "✅ Build Successful!"
    echo "📄 BIN file: $TARGET_BIN"
    
    # Check if flash requested
    if [ "$1" == "flash" ]; then
        echo "⚡ Starting Flash process..."
        st-flash write $TARGET_BIN $START_ADDRESS
        if [ $? -eq 0 ]; then
            echo "🎉 Flash Complete!"
        else
            echo "❌ Flash Failed! Check ST-Link connection."
        fi
    fi
    
    if [ "$1" == "clean" ]; then
        echo "🧹 Cleaning..."
        make clean
    fi
else
    echo "❌ Build Failed! Please check the errors above."
fi
