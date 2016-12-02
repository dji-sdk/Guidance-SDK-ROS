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

ros::Publisher depth_image_pub;
ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher imu_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher velocity_pub;
ros::Publisher ultrasonic_pub;

using namespace cv;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key       = 0;
bool            show_images = 0;
uint8_t         verbosity = 0;
e_vbus_index	CAMERA_ID = e_vbus1;
DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat				g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
Mat				g_depth(HEIGHT,WIDTH,CV_16SC1);
Mat				depth8(HEIGHT, WIDTH, CV_8UC1);

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

		if ( data->m_greyscale_image_left[CAMERA_ID] ){
			memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE);
            if (show_images) {
			    imshow("left",  g_greyscale_image_left);
            }
			// publish left greyscale image
			cv_bridge::CvImage left_8;
			g_greyscale_image_left.copyTo(left_8.image);
			left_8.header.frame_id  = "guidance";
			left_8.header.stamp	= ros::Time::now();
			left_8.encoding		= sensor_msgs::image_encodings::MONO8;
			left_image_pub.publish(left_8.toImageMsg());
		}
		if ( data->m_greyscale_image_right[CAMERA_ID] ){
			memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE);
            if (show_images) {
			    imshow("right", g_greyscale_image_right);
            }
			// publish right greyscale image
			cv_bridge::CvImage right_8;
			g_greyscale_image_right.copyTo(right_8.image);
			right_8.header.frame_id  = "guidance";
			right_8.header.stamp	 = ros::Time::now();
			right_8.encoding  	 = sensor_msgs::image_encodings::MONO8;
			right_image_pub.publish(right_8.toImageMsg());
		}
		if ( data->m_depth_image[CAMERA_ID] ){
			memcpy(g_depth.data, data->m_depth_image[CAMERA_ID], IMAGE_SIZE * 2);
			g_depth.convertTo(depth8, CV_8UC1);
            if (show_images) {
			    imshow("depth", depth8);
            }
			//publish depth image
			cv_bridge::CvImage depth_16;
			g_depth.copyTo(depth_16.image);
			depth_16.header.frame_id  = "guidance";
			depth_16.header.stamp	  = ros::Time::now();
			depth_16.encoding	  = sensor_msgs::image_encodings::MONO16;
			depth_image_pub.publish(depth_16.toImageMsg());
		}
		
        key = waitKey(1);
    }

    /* imu */
    if ( e_imu == data_type && NULL != content )
    {
        imu *imu_data = (imu*)content;
        if (verbosity > 1) {
            printf( "frame index: %d, stamp: %d\n", imu_data->frame_index, imu_data->time_stamp );
            printf( "imu: [%f %f %f %f %f %f %f]\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3] );
 	
        }
    	// publish imu data
		geometry_msgs::TransformStamped g_imu;
		g_imu.header.frame_id = "guidance";
		g_imu.header.stamp    = ros::Time::now();
		g_imu.transform.translation.x = imu_data->acc_x;
		g_imu.transform.translation.y = imu_data->acc_y;
		g_imu.transform.translation.z = imu_data->acc_z;
		g_imu.transform.rotation.w = imu_data->q[0];
		g_imu.transform.rotation.x = imu_data->q[1];
		g_imu.transform.rotation.y = imu_data->q[2];
		g_imu.transform.rotation.z = imu_data->q[3];
		imu_pub.publish(g_imu);
    }
    /* velocity */
    if ( e_velocity == data_type && NULL != content )
    {
        velocity *vo = (velocity*)content;
        if (verbosity > 1) {
            printf( "frame index: %d, stamp: %d\n", vo->frame_index, vo->time_stamp );
            printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
        }
	
		// publish velocity
		geometry_msgs::Vector3Stamped g_vo;
		g_vo.header.frame_id = "guidance";
		g_vo.header.stamp    = ros::Time::now();
		g_vo.vector.x = 0.001f * vo->vx;
		g_vo.vector.y = 0.001f * vo->vy;
		g_vo.vector.z = 0.001f * vo->vz;
		velocity_pub.publish(g_vo);
    }

    /* obstacle distance */
    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;
        if (verbosity > 1) { 
            printf( "frame index: %d, stamp: %d\n", oa->frame_index, oa->time_stamp );
            printf( "obstacle distance:" );
            for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
            {
                printf( " %f ", 0.01f * oa->distance[i] );
            }
            printf( "\n" );
        }

		// publish obstacle distance
		sensor_msgs::LaserScan g_oa;
		g_oa.ranges.resize(CAMERA_PAIR_NUM);
		g_oa.header.frame_id = "guidance";
		g_oa.header.stamp    = ros::Time::now();
		for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
			g_oa.ranges[i] = 0.01f * oa->distance[i];
		obstacle_distance_pub.publish(g_oa);
	}

    /* ultrasonic */
    if ( e_ultrasonic == data_type && NULL != content )
    {
        ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
        if (verbosity > 1) {
            printf( "frame index: %d, stamp: %d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
            for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
            {
                printf( "ultrasonic distance: %f, reliability: %d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d] );
            }
        }
	
		// publish ultrasonic data
		sensor_msgs::LaserScan g_ul;
		g_ul.ranges.resize(CAMERA_PAIR_NUM);
		g_ul.intensities.resize(CAMERA_PAIR_NUM);
		g_ul.header.frame_id = "guidance";
		g_ul.header.stamp    = ros::Time::now();
		for ( int d = 0; d < CAMERA_PAIR_NUM; ++d ){
			g_ul.ranges[d] = 0.001f * ultrasonic->ultrasonic[d];
			g_ul.intensities[d] = 1.0 * ultrasonic->reliability[d];
		}
		ultrasonic_pub.publish(g_ul);
    }

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
        verbosity = 2;
    }
	if(argc==2 && !strcmp(argv[1], "h")){
		printf("This is demo program showing data from Guidance.\n\t" 
			" 'a','d','w','s','x' to select sensor direction.\n\t"
			" 'j','k' to change the exposure parameters.\n\t"
			" 'm' to switch between AEC and constant exposure modes.\n\t"
			" 'n' to return to default exposure mode and parameters.\n\t"
			" 'q' to quit.");
		return 0;
	}
	
    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;
    depth_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/depth_image",1);
    left_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/left_image",1);
    right_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/right_image",1);
    imu_pub  				= my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
    velocity_pub  			= my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
    obstacle_distance_pub	= my_node.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance",1);
    ultrasonic_pub			= my_node.advertise<sensor_msgs::LaserScan>("/guidance/ultrasonic",1);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);

	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	RETURN_IF_ERR(err_code);
    std::cout<<"Sensor online status: ";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
        std::cout<<online_status[i]<<" ";
    std::cout<<std::endl;

	// get cali param
	stereo_cali cali[CAMERA_PAIR_NUM];
	err_code = get_stereo_cali(cali);
	RETURN_IF_ERR(err_code);
    std::cout<<"cu\tcv\tfocal\tbaseline\n";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
	{
        std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
	}
	
    /* select data */
    err_code = select_greyscale_image(CAMERA_ID, true);
	RETURN_IF_ERR(err_code);
    err_code = select_greyscale_image(CAMERA_ID, false);
	RETURN_IF_ERR(err_code);
    err_code = select_depth_image(CAMERA_ID);
	RETURN_IF_ERR(err_code);
    select_imu();
    select_ultrasonic();
    select_obstacle_distance();
    select_velocity();
    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);
	
	// for setting exposure
	exposure_param para;
	para.m_is_auto_exposure = 1;
	para.m_step = 10;
	para.m_expected_brightness = 120;
    para.m_camera_pair_index = CAMERA_ID;
	
	std::cout << "start_transfer" << std::endl;

	while (ros::ok())
	{
		g_event.wait_event();
		if (key > 0){
			// set exposure parameters
			if(key=='j' || key=='k' || key=='m' || key=='n'){
				if(key=='j'){
					if(para.m_is_auto_exposure) para.m_expected_brightness += 20;
					else para.m_exposure_time += 3;
				}
				else if(key=='k'){
					if(para.m_is_auto_exposure) para.m_expected_brightness -= 20;
					else para.m_exposure_time -= 3;
				}
				else if(key=='m'){
					para.m_is_auto_exposure = !para.m_is_auto_exposure;
                    std::cout<<"exposure is "<<para.m_is_auto_exposure<<std::endl;
				}
				else if(key=='n'){//return to default
					para.m_expected_brightness = para.m_exposure_time = 0;
				}

                std::cout<<"Setting exposure parameters....SensorId="<<CAMERA_ID<<std::endl;
                para.m_camera_pair_index = CAMERA_ID;
				set_exposure_param(&para);
				key = 0;
			}
			else if (key == 'q' || key == 'w' || key == 'd' || key == 'x' || key == 'a' || key == 's'){// switch image direction
				err_code = stop_transfer();
				RETURN_IF_ERR(err_code);
				reset_config();

				if (key == 'q') break;
				if (key == 'w') CAMERA_ID = e_vbus1;
				if (key == 'd') CAMERA_ID = e_vbus2;
				if (key == 'x') CAMERA_ID = e_vbus3;
				if (key == 'a') CAMERA_ID = e_vbus4;	   
				if (key == 's') CAMERA_ID = e_vbus5;

				select_greyscale_image(CAMERA_ID, true);
				select_greyscale_image(CAMERA_ID, false);
				select_depth_image(CAMERA_ID);

                select_imu();
                select_ultrasonic();
                select_obstacle_distance();
                select_velocity();

				err_code = start_transfer();
				RETURN_IF_ERR(err_code);
				key = 0;
            }
		}
        ros::spinOnce();
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
