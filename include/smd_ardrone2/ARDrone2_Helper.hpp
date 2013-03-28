#ifndef _ARDrone2_Helper_hpp
#define _ARDrone2_Helper_hpp

#include "smd_ardrone2/DroneAnimate.h"
#include "smd_ardrone2/DroneSetCam.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_srvs/Empty.h>

namespace smd_ardrone2
{
	class ARDrone2_Helper : public nodelet::Nodelet
	{
	private:
		virtual void onInit( );

		bool FlipForwardCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & );
		bool FlipBackwardCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & );
		bool FlipLeftCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & );
		bool FlipRightCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & );
		bool ToggleCamCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & );

		ros::ServiceServer flip_forward_srv;
		ros::ServiceServer flip_backward_srv;
		ros::ServiceServer flip_left_srv;
		ros::ServiceServer flip_right_srv;
		ros::ServiceServer togglecam_srv;
		ros::ServiceClient flip_srv;
		ros::ServiceClient cam_srv;

		smd_ardrone2::DroneAnimate flip_srv_msg;
		smd_ardrone2::DroneSetCam cam_srv_msg;
	};
}

#endif /* _ARDrone2_Helper_hpp */
