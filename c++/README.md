To use MercurySDK ver. c++ in Arduino version,

1. Remove "port_handler_linux.cpp" and "port_handler_windows.cpp" in /src/mercury_sdk/
2. Remove not related sources
  * CHANGELOG.rst
  * CMakeLists.txt
  * package.xml
3. Change folder name from "c++" to "MercurySDK"
4. Put newly made "MercurySDK" folder into the libraries/ of Arduino IDE
5. MercurySDK example for Arduino can be referred in the [OpenCR Repository](https://github.com/ROBOTIS-GIT/OpenCR/tree/master/arduino/opencr_arduino/opencr/libraries/OpenCR/examples/07.%20MercurySDK)

To use MercurySDK ver. c++ in ROS package,

1. Remove "port_handler_arduino.cpp" and "port_handler_windows.cpp" in /src/mercury_sdk/
2. Remove not releated sources
  * library.properties
  * keywords.txt
3. (Remove all sources in the parent folder, but actually it doesn't matter.)
4. Put whole package into appropriate location, then catkin make, or sudo apt-get install ros-{version}-mercury-sdk
