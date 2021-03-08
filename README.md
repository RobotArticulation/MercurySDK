![RobotArticulation logo](http://robotarticulation.com/img/logos/RobotArticulation_header.png?format=1000w)

# MercurySDK
This is the repository for the software development kit (SDK) for the Mercury range of digital servos

Please note that this SDK is based on the Dynamixel SDK. Please see the copyright notices in the source file headers.

<H3><ins>Installation</ins></H3>

Clone the [MercurySDK](https://github.com/RobotArticulation/MercurySDK)

### Build and install the MercurySDK: ###

#### Linux (x64) ####
- cd MercurySDK/c++/build/linux64
- make
- sudo make install

#### Windows (x64) ####
- in Visual Studio, open the solution file MercurySDK/c++/build/win64/mcy_x64_cpp.sln
- build-all in the release configuration.

<H3><ins>Running the examples</ins></H3>

### Building and running the ping example: ###

#### Linux (x64) ####
- cd MercurySDK/c++/example/ping/linux64
- ensure that the USB device name in ping.cpp correctly identifies the USB2Mercury device's USB port
- make
- ./ping

#### Windows (x64) ####
- in Visual Studio, open the solution file MercurySDK/c++/example/ping/win64/ping.sln
- ensure that the USB device name in ping.cpp correctly identifies the USB2Mercury device's USB port
- build-all in the release configuration.
- ensure that mcy_x64_cpp.dll is in the PATH
- run ping.exe