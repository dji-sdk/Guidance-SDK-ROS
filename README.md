# Gudiance-SDK-ROS
The official ROS package of Guidance SDK for 32/64 bit Ubuntu and XU3.

- We write the CMakeLists.txt file so that it automatically detects your operating system and choose the appropriate library file.
- We suppose the users are using USB for Guidance SDK on ROS. To use UART for Guidance SDK, plese reference [uart_example](https://github.com/dji-sdk/GuidanceSDK/tree/master/examples/uart_example).

# How to use
1. Setup USB devide rules so that no root privilege is required when using Guidance SDK via USB.
		
		sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"fff0\", ATTR{idProduct}==\"d009\", MODE=\"0666\"" > /etc/udev/rules.d/51-guidance.rules'
2. Clone the repo to the catkin workspace source directory `catkin_ws/src` and then 
	
		cd ~/catkin_ws
		catkin_make
		rosrun guidance guidanceNode
		rosrun guidance guidanceNodeTest

# Documentation
To reduce the size of this package, we omit all documents. 

- For getting started, please refer to [Developer Guide](https://developer.dji.com/guidance-sdk/documentation/application-development-guides/index.html).
- For detailed API documentation, please refer to [Guidance_SDK_API](https://developer.dji.com/guidance-sdk/documentation/introduction/index.html).

# Using ROS tools for calibration and stereo processing
(experimental node by [@madratman](https://github.com/madratman/). Ideal would be using [camera_info_manager](http://wiki.ros.org/camera_info_manager) on the lines on [camera1394stereo](http://wiki.ros.org/camera1394stereo))


- Look inside `/calibration_files`. A sample file for one stereo pair is provided. 
ROS image pipeline needs a `camera_info` msg which consists of the calibration parameters. 
`guidanceNodeCalibration` is an experimental node that parses the calibration params from the YAMLs in the `/calibration_files directory`, publishes on the `/guidance/right/camera_info` and `/guidance/left/camera_info` topics. 

- First, you should calibrate using the [camera_calibration](http://wiki.ros.org/camera_calibration) package, and save the result to the left and right YAML in `/calibration_files` directory. 
```
roslaunch guidance load_calib_file.launch  
(The launch file just sets a couple of parameters to retrieve the calibration files)

rosrun guidance guidanceNodeCalibration  

 rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 right:=/guidance/right/image_raw left:=/guidance/left/image_raw right_camera:=/guidance/right left_camera:=/guidance/left --no-service-check
```        
- Follow the calibration tutorials [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) and [here](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)   
If you are unable to save the calibration file using the GUI, you can do it manually from the terminal output. A reference for the same is provided in the sample `/calibration_files/raw_from_terminal` file. 
 
- Alternatively if you don't want to recalibrate, you can also manually enter the current calibration params in the YAML, which you would have from using the DJI Windows utility for Guidance. The same is printed out in the terminal from either node - the official `guidanceNode` or the experimental `guidanceNodeCalibration`.  

- Now that calibration is done, or you chose to use enter the pre-existing params in the YAMLs, we can use `stereo_image_proc` and play around to view and improve the disparity and point cloud in RViz.   
We can use the dynamic reconfigure GUI to change the stereo algo used and its params as explained in [this tutorial](http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters). 

`ROS_NAMESPACE=guidance rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True`   
`rosrun image_view stereo_view stereo:=guidance image:=image_rect_color`   
`rosrun rqt_reconfigure rqt_reconfigure `     
`rosrun rviz rviz ` Change frame to "guidance". Add published point cloud pc2.
