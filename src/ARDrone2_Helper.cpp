#include "smd_ardrone2/ARDrone2_Helper.hpp"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( smd_ardrone2, ARDrone2_Helper, smd_ardrone2::ARDrone2_Helper, nodelet::Nodelet )

namespace smd_ardrone2
{
	void ARDrone2_Helper::onInit( )
	{
		ros::NodeHandle nh = getNodeHandle( );

		flip_forward_srv = nh.advertiseService( "flip_forward", &ARDrone2_Helper::FlipForwardCB, this );
		flip_backward_srv = nh.advertiseService( "flip_backward", &ARDrone2_Helper::FlipBackwardCB, this );
		flip_left_srv = nh.advertiseService( "flip_left", &ARDrone2_Helper::FlipLeftCB, this );
		flip_right_srv = nh.advertiseService( "flip_right", &ARDrone2_Helper::FlipRightCB, this );
		togglecam_srv = nh.advertiseService( "togglecam", &ARDrone2_Helper::ToggleCamCB, this );
		flip_srv = nh.serviceClient<smd_ardrone2::DroneAnimate>("animate");
		cam_srv = nh.serviceClient<smd_ardrone2::DroneSetCam>("set_cam");

		flip_srv_msg.request.dur = -1;
		cam_srv_msg.request.num = 0;
	}

	bool ARDrone2_Helper::FlipForwardCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		flip_srv_msg.request.num = 16;
		return flip_srv.call( flip_srv_msg );
	}

	bool ARDrone2_Helper::FlipBackwardCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		flip_srv_msg.request.num = 17;
		return flip_srv.call( flip_srv_msg );
	}

	bool ARDrone2_Helper::FlipLeftCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		flip_srv_msg.request.num = 18;
		return flip_srv.call( flip_srv_msg );
	}

	bool ARDrone2_Helper::FlipRightCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		flip_srv_msg.request.num = 19;
		return flip_srv.call( flip_srv_msg );
	}

	bool ARDrone2_Helper::ToggleCamCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		cam_srv_msg.request.num = ( cam_srv_msg.request.num ) ? 0 : 1;
		return cam_srv.call( cam_srv_msg );
	}
}

