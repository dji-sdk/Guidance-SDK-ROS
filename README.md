# GudianceSDKROS
The official ROS package of Guidance SDK for 32/64 bit Ubuntu and XU3.

We write the CMakeLists.txt file so that it automatically detects your operating system and choose the appropriate library file.

# How to use
Clone the repo to the catkin workspace source directory `catkin_ws/src` and then 
	
	>> cd ~/catkin_ws
	>> catkin_make
	>> rosrun GuidanceRos GuidanceNode

# Documentation
To reduce the size of this package, we omit all documents. Please refer to [Guidance_SDK_API](https://github.com/dji-sdk/GuidanceSDK/blob/master/doc/Guidance_SDK_API.md) for detailed documentation.

For saving the trouble of having to use `sudo` every time running Guidance SDK programs, please refer to [Guidance_SDK_API](https://github.com/dji-sdk/GuidanceSDK/blob/master/doc/Guidance_SDK_API.md) to add rules to `/etc/udev/rules.d/` to allow access of Guidance for ordinary users.