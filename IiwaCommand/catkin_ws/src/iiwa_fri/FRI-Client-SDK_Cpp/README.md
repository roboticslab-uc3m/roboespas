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

## Examples
For example, to try ```IOAccess example```, after compiling, go to ```...\example\IOAccess``` and run 
```
./IOAccess “192.170.10.2” 30200
```
Change the IP and port for the IP and port peviously set up in the IIWA cabinet. Run the corresponding example in the IIWA SmartPad, if it worked, it should appear in the Ubuntu terminal:
```
IOAccessClient initialized: Enter IOAccess Client Application Exit IOAccess Client Application 
```
And in the SmartPad: 
```
Creating FRI connection from controller port 30200 to 192.170.10.200:30200 SendPeriod: 5ms | ReceiveMultiplier: 1 Connection to Client established Enable clock Do something ... Disable clock Close connection to client 
```
