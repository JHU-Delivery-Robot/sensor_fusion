# Sensor fusion

Sensor fusion firmware for JHU Deliverbot project

# Installing & Building

Only Linux is support for building.

You will need to install the GCC ARM compilers, CMake, and Git. The GCC ARM compilers can be install via
```
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

Then clone this repository with submodules:
```
git clone --recurse-submodules https://github.com/JHU-Delivery-Robot/sensor-fusion.git
```
and from inside of `./sensor-fusion/pico-sdk` run `git submodule update --init`.

Next, configure CMake with
```
mkdir build
cd build
cmake ..
```
and build with
```
make all
```
