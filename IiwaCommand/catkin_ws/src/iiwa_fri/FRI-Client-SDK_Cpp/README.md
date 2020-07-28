# FRI-Client-SDK_Cpp

Official KUKA Fast Robot Interface C++ SDK ported to the CMake build system.

## Contents

- cmake: CMake modules
- doc: FRI client SDK documentation
- example: Example applications
- include: FRI client SDK headers needed to write FRI client applications (see examples)
- src: FRI client sources

## Installation

On Linux:

```cmake
mkdir build && cd build
cmake ..
make -j$(nproc)
make -j$(nproc) examples # in case you want to compile examples
sudo make install
```
