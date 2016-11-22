/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic


 using namespace cv;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)


 ros::Publisher image_pubs[10];
 cv_bridge::CvImage images[10];
 Mat greyscales[10] {Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),
                    Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1)};

 char       key         = 0;
 bool       show_images = 0;
 DJI_lock   g_lock;
 DJI_event  g_event;

 std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value){
   const char* s = 0;
   static char str[100]={0};
#define PROCESS_VAL(p) case(p): s = #p; break;
   switch(value){
    PROCESS_VAL(e_OK);     
    PROCESS_VAL(e_load_libusb_err);     
    PROCESS_VAL(e_sdk_not_inited);
    PROCESS_VAL(e_disparity_not_allowed);
    PROCESS_VAL(e_image_frequency_not_allowed);
    PROCESS_VAL(e_config_not_ready);
    PROCESS_VAL(e_online_flag_not_ready);
    PROCESS_VAL(e_stereo_cali_not_ready);
    PROCESS_VAL(e_libusb_io_err);
    PROCESS_VAL(e_timeout);
    default:
    strcpy(str, "Unknown error");
    s = str;
    break;
  }
#undef PROCESS_VAL
  return out << s;
}

int my_callback(int data_type, int data_len, char *content)
{
  g_lock.enter();

    /* image data */
  if (e_image == data_type && NULL != content)
  {        
    image_data* data = (image_data*)content;
    printf("hey");
    for (int i = 0; i < 5; ++i)
    {
      int j = 2*i;
      if(data->m_greyscale_image_left[i]){
        memcpy(greyscales[j].data, data->m_greyscale_image_left[i], IMAGE_SIZE);
		greyscales[j].copyTo(images[j].image);
        images[j].header.stamp  = ros::Time::now();
        images[j].encoding    = sensor_msgs::image_encodings::MONO8;
        image_pubs[j].publish(images[j].toImageMsg());
      }
      if(data->m_greyscale_image_right[i]){
        memcpy(greyscales[j+1].data, data->m_greyscale_image_right[i], IMAGE_SIZE);
		greyscales[j+1].copyTo(images[j+1].image);
        images[j+1].header.stamp = ros::Time::now();
        images[j+1].encoding = sensor_msgs::image_encodings::MONO8;
        image_pubs[j+1].publish(images[j+1].toImageMsg());
      }
    }

  }

  key = waitKey(1);

  g_lock.leave();
  g_event.set_event();

  return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

int main(int argc, char** argv)
{
  if (argc < 2) {
    show_images = true;
  }
  if(argc>=2 && !strcmp(argv[1], "h")){
    printf("This program publish all the cameras in topics \n from /guidance/cameras/0/left to /guidance/cameras/4/right\n"
           "press 'q' to quit.");
    return 0;
  }

  /* initialize ros */
  ros::init(argc, argv, "GuidanceCamerasOnly");
  ros::NodeHandle my_node;

  image_pubs[0] = my_node.advertise<sensor_msgs::Image>("/guidance/0/left",1);
  image_pubs[1] = my_node.advertise<sensor_msgs::Image>("/guidance/0/right",1);
  image_pubs[2] = my_node.advertise<sensor_msgs::Image>("/guidance/1/left",1);
  image_pubs[3] = my_node.advertise<sensor_msgs::Image>("/guidance/1/right",1);
  image_pubs[4] = my_node.advertise<sensor_msgs::Image>("/guidance/2/left",1);
  image_pubs[5] = my_node.advertise<sensor_msgs::Image>("/guidance/2/right",1);
  image_pubs[6] = my_node.advertise<sensor_msgs::Image>("/guidance/3/left",1);
  image_pubs[7] = my_node.advertise<sensor_msgs::Image>("/guidance/3/right",1);
  image_pubs[8] = my_node.advertise<sensor_msgs::Image>("/guidance/4/left",1);
  image_pubs[9] = my_node.advertise<sensor_msgs::Image>("/guidance/4/right",1);

  images[0].header.frame_id = "guidanceDown";
  images[1].header.frame_id = "guidanceDown";
  images[2].header.frame_id = "guidanceFront";
  images[3].header.frame_id = "guidanceFront";
  images[4].header.frame_id = "guidanceRight";
  images[5].header.frame_id = "guidanceRight";
  images[6].header.frame_id = "guidanceRear";
  images[7].header.frame_id = "guidanceRear";
  images[8].header.frame_id = "guidanceLeft";
  images[9].header.frame_id = "guidanceLeft";

  /* initialize guidance */
  reset_config();
  int err_code = init_transfer();
  RETURN_IF_ERR(err_code);

  int online_status[5];
  err_code = get_online_status(online_status);
  RETURN_IF_ERR(err_code);
  std::cout<<"Sensor online status: ";
  for (int i=0; i<5; i++)
    std::cout<<online_status[i]<<" ";
  std::cout<<std::endl;

  // get cali param
  stereo_cali cali[5];
  err_code = get_stereo_cali(cali);
  RETURN_IF_ERR(err_code);
  std::cout<<"cu\tcv\tfocal\tbaseline\n";
  for (int i=0; i<5; i++)
  {
    std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
  }

  /* select data */
  err_code = select_greyscale_image(e_vbus1, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus1, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus2, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus2, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus3, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus3, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus4, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus4, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus5, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus5, false);
  RETURN_IF_ERR(err_code);
  /* start data transfer */
  err_code = set_sdk_event_handler(my_callback);
  RETURN_IF_ERR(err_code);
  err_code = start_transfer();
  RETURN_IF_ERR(err_code);

  std::cout << "start_transfer" << std::endl;

  while (ros::ok())
  {
    g_event.wait_event();
    ros::spinOnce();
    if (key > 0){
     if (key == 'q'){
        err_code = stop_transfer();
		RETURN_IF_ERR(err_code);
		reset_config();
      break;
    }
  }
}

	/* release data transfer */
err_code = stop_transfer();
RETURN_IF_ERR(err_code);
	//make sure the ack packet from GUIDANCE is received
sleep(1);
std::cout << "release_transfer" << std::endl;
err_code = release_transfer();
RETURN_IF_ERR(err_code);

return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
