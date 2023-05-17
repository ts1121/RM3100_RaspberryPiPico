#!/bin/bash
cp ../pico-sdk/external/pico_sdk_import.cmake .
rm -r build
mkdir build
cd build
export PICO_SDK_PATH=../../pico-sdk
cmake ..
make

wslfile="/home/ts1121/pi/pico/rm3100_i2c/build/rm3100_i2c.uf2"
winfile="/mnt/c/Users/talha/rm3100_i2c.uf2"
picodrive="D:"

cp "$wslfile" "$winfile"
powershell.exe -Command "Copy-Item -Path 'C:\Users\talha\rm3100_i2c.uf2' -Destination '$picodrive'"