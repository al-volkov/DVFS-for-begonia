#! /bin/bash

export PATH="$(pwd)/proton-clang/bin:$PATH"
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export CROSS_COMPILE_ARM32=arm-linux-gnueabi-

make O=out begonia_user_defconfig
make -j$(nproc --all) O=out CC=clang
