#include "smd_ardrone2/ARDrone2_Control.hpp"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>

#include <boost/thread.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

PLUGINLIB_DECLARE_CLASS( smd_ardrone2, ARDrone2_Control, smd_ardrone2::ARDrone2_Control, nodelet::Nodelet )

namespace smd_ardrone2
{
	ARDrone2_Control::ARDrone2_Control( ) :
		sockfd_in( -1 ),
		sockfd_out( -1 ),
		sockaddr( "192.168.1.1" ),
		sockto( 0.5 ),
		sockcool( 0.5 ),
		have_config( false ),
		altitude_max( 3 )
	{
		memset( &last_hdr, 0, sizeof( last_hdr ) );

		global_send.unlock( );
	}

	ARDrone2_Control::~ARDrone2_Control( )
	{
		disconnect( );
		NODELET_DEBUG( "ARDrone2_Control: Killing threads..." );
		spin_thread.interrupt( );
		if( spin_thread.joinable( ) )
			spin_thread.join( );
	}

	void ARDrone2_Control::onInit( )
	{
		ros::NodeHandle nh = getNodeHandle( );
		ros::NodeHandle priv_nh = getPrivateNodeHandle( );

		priv_nh.param( "ardrone_addr", sockaddr, (std::string)"192.168.1.1" );
		priv_nh.param( "sock_timeout", sockto, 0.5 );
		priv_nh.param( "sock_cooldown", sockcool, 0.5 );
		priv_nh.param( "altitude_max", altitude_max, 3.0 );

		spin_thread = boost::thread( &ARDrone2_Control::spin, this );
	}

	bool ARDrone2_Control::connect( )
	{
		int ret;
		struct addrinfo hints, *servinfo = NULL, *p = NULL;
		struct timeval tv;

		disconnect( );

		memset( &hints, 0, sizeof hints );
		hints.ai_family = AF_INET;		// IPv4 Only (for now)
		hints.ai_socktype = SOCK_DGRAM;
		hints.ai_protocol = IPPROTO_UDP;

		tv.tv_sec   = (int)sockto;
		tv.tv_usec  = ( sockto - tv.tv_sec ) * 1000000;

		if( ( ret = getaddrinfo( sockaddr.c_str( ), "5554", &hints, &servinfo ) ) != 0 )
		{
			NODELET_DEBUG( "ARDrone2_Control: Failed to connect to inbound: %s", gai_strerror( ret ) );
			return false;
		}

		for( p = servinfo; p != NULL; p = p->ai_next )
		{
			if( ( sockfd_in = socket( p->ai_family, p->ai_socktype, p->ai_protocol ) ) == -1 )
			{
				NODELET_DEBUG( "ARDrone2_Control: Skipping invalid socket" );
				continue;
			}

			if( setsockopt( sockfd_in, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( tv ) ) )
			{
				NODELET_DEBUG( "ARDrone2_Control: Failed to set socket receive timeout" );
				close( sockfd_in );
				sockfd_in = -1;
				continue;
			}

			if( setsockopt( sockfd_in, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof( tv ) ) )
			{
				NODELET_DEBUG( "ARDrone2_Control: Failed to set socket send timeout" );
				close( sockfd_in );
				sockfd_in = -1;
				continue;
			}

			if ( ::connect( sockfd_in, p->ai_addr, p->ai_addrlen ) == -1 )
			{
				NODELET_DEBUG( "ARDrone2_Control: Connection attempt failed" );
				close( sockfd_in );
				sockfd_in = -1;
				continue;
			}

			break;
		}

		freeaddrinfo( servinfo );

		if( p == NULL )
		{
			NODELET_ERROR( "ARDrone2_Control: Inbound connection failure" );
			return false;
		}

		if( ( ret = getaddrinfo( sockaddr.c_str( ), "5556", &hints, &servinfo ) ) != 0 )
		{
			NODELET_DEBUG( "ARDrone2_Control: Failed to connect to outbound: %s", gai_strerror( ret ) );
			return false;
		}

		for( p = servinfo; p != NULL; p = p->ai_next )
		{
			if( ( sockfd_out = socket( p->ai_family, p->ai_socktype, p->ai_protocol ) ) == -1 )
			{
				NODELET_DEBUG( "ARDrone2_Control: Skipping invalid socket" );
				continue;
			}

			if( setsockopt( sockfd_out, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( tv ) ) )
			{
				NODELET_DEBUG( "ARDrone2_Control: Failed to set socket receive timeout" );
				close( sockfd_out );
				sockfd_out = -1;
				continue;
			}

			if( setsockopt( sockfd_out, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof( tv ) ) )
			{
				NODELET_DEBUG( "ARDrone2_Control: Failed to set socket send timeout" );
				close( sockfd_out );
				sockfd_out = -1;
				continue;
			}

			if ( ::connect( sockfd_out, p->ai_addr, p->ai_addrlen ) == -1 )
			{
				NODELET_DEBUG( "ARDrone2_Control: Connection attempt failed" );
				close( sockfd_out );
				sockfd_out = -1;
				continue;
			}

			break;
		}

		freeaddrinfo( servinfo );

		if( p == NULL )
		{
			NODELET_ERROR( "ARDrone2_Control: Outbound connection failure" );
			disconnect( );
			return false;
		}

		if( !sendWake( ) )
		{
			NODELET_ERROR( "ARDrone2_Control: Failed to send wake" );
			disconnect( );
			return false;
		}

		sequence = 0;

		NODELET_INFO( "ARDrone2_Control: Initialized - Waiting for handshake" );

		return true;
	}

	void ARDrone2_Control::disconnect( )
	{
		close( sockfd_in );
		sockfd_in = -1;
		close( sockfd_out );
		sockfd_out = -1;
		if( twist_sub )
			twist_sub.shutdown( );
		if( imu_pub )
			imu_pub.shutdown( );
		if( sonar_pub )
			sonar_pub.shutdown( );
		if( twist_pub )
			twist_pub.shutdown( );
		if( takeoff_ser )
			takeoff_ser.shutdown( );
		if( land_ser )
			land_ser.shutdown( );
		if( trim_ser )
			trim_ser.shutdown( );
		if( reset_ser )
			reset_ser.shutdown( );
		if( cfg_ser )
			cfg_ser.shutdown( );
		if( ledanim_ser )
			ledanim_ser.shutdown( );
		if( anim_ser )
			anim_ser.shutdown( );
		if( set_cam_ser )
			set_cam_ser.shutdown( );
	}

	void ARDrone2_Control::spinOnce( )
	{
		static uint8_t pkt[16384];
		static struct navdata *hdr = (struct navdata *)pkt;
		static int bytes;

		if( sockfd_in == -1 )
		{
			if( !connect( ) )
			{
				NODELET_INFO( "ARDrone2_Control: Cooling off for %1.1fs", sockcool );
				usleep( sockcool * 1000000 );
				return;
			}
		}

		if( ( bytes = recv( sockfd_in, &pkt, sizeof( pkt ), 0 ) ) == -1 )
		{
			NODELET_WARN( "ARDrone2_Control: Timeout receiving data" );
			disconnect( );
			return;
		}

		if( bytes >= 24 && hdr->header == 0x55667788 )
		{
			uint8_t *buff = (uint8_t *)hdr + sizeof( *hdr );
			uint8_t *op;
			std::vector<struct navdata_option> opts;

			do
			{
				struct navdata_option opt;
				op = (uint8_t *)&opt;

				memcpy( op, buff, sizeof( opt.tag ) + sizeof( opt.size ) );
				buff += sizeof( opt.tag ) + sizeof( opt.size );
				op += sizeof( opt.tag ) + sizeof( opt.size );

				memcpy( op, buff, opt.size - ( sizeof( opt.tag ) + sizeof( opt.size ) ) );
				buff += opt.size - ( sizeof( opt.tag ) + sizeof( opt.size ) );

				opts.push_back( opt );
			} while( opts.back( ).tag != NAVDATA_CKS_TAG && buff < pkt + sizeof( pkt ) );

			if( buff >= pkt + sizeof( pkt ) )
			{
				NODELET_WARN( "ARDrone2_Control: Buffer read overflow" );
				disconnect( );
				return;
			}

			uint32_t sum = 0;
			for( unsigned int i = 0; i < bytes - ( sizeof( opts.back( ).tag ) + sizeof( opts.back( ).size ) + sizeof( opts.back( ).data.cks_payload ) ); i++ )
				sum += (uint32_t)pkt[i];

			if( sum != opts.back( ).data.cks_payload.cks )
			{
				NODELET_WARN( "ARDrone2_Control: Checksum failure on #%d", hdr->sequence );
				return;
			}

			if( hdr->sequence <= last_hdr.sequence )
			{
				NODELET_WARN( "ARDrone2_Control: Navdata sequence error (%d before %d)", hdr->sequence, last_hdr.sequence );
				return;
			}

			NODELET_DEBUG( "ARDrone2_Control: Found %zu navdata options", opts.size( ) );

			processNavdata( *hdr );
			processNavdataOptions( opts );
		}
		else
			NODELET_WARN( "ARDrone2_Control: Skipping %d bytes (Bad Header)", bytes );
	}

	void ARDrone2_Control::spin( )
	{
		while( true )
		{
			boost::this_thread::interruption_point( );
			spinOnce( );
		}
	}

	void ARDrone2_Control::dumpHeader( const struct navdata *hdr ) const
	{
		std::cerr << "ARDrone2_Control: Header" << std::endl
			<< "Header: " << (unsigned int)hdr->header << std::endl
			<< "State: " << (unsigned int)hdr->ardrone_state << std::endl
			<< "Sequence: " << (unsigned int)hdr->sequence << std::endl
			<< "Vision: " << (unsigned int)hdr->vision_defined << std::endl
			<< std::endl;
	}

	bool ARDrone2_Control::sendWake( )
	{
		NODELET_DEBUG( "ARDrone2_Control: Sending wake packet" );
		int data = 1;
		if( sockfd_in != -1 )
		{
			boost::mutex::scoped_lock scoped_lock( global_send );
			if( send( sockfd_in, &data, sizeof( data ), 0 ) == sizeof( data ) )
				return true;
		}
		return false;
	}

	bool ARDrone2_Control::sendConfig( )
	{
		NODELET_DEBUG( "ARDrone2_Control: Sending config" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			static const uint32_t navmask = ( 1 << NAVDATA_DEMO_TAG ) | ( 1 << NAVDATA_RAW_MEASURES_TAG );
			boost::mutex::scoped_lock scoped_lock( global_config );
			boost::mutex::scoped_lock scoped_lock2( global_send );
			buf << "AT*CONFIG=" << ++sequence << ",\"general:navdata_demo\",\"FALSE\"" << '\r';
			buf << "AT*CONFIG=" << ++sequence << ",\"general:navdata_options\",\"" << navmask << "\"" << '\r';
			//buf << "AT*CONFIG=" << ++sequence << ",\"video:video_codec\",\"" << H264_360P_CODEC << '\"' << '\r';
			buf << "AT*CONFIG=" << ++sequence << ",\"control:altitude_max\",\"" << (int)( altitude_max * 1000 + 0.5 ) << '\"' << '\r';
			ROS_WARN_COND( buf.str( ).length( ) > 1024, "ARDrone2_Control: Configuration is over 1024 bytes (%zu)", buf.str( ).length( ) );
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
				return true;
		}
		return false;
	}

	bool ARDrone2_Control::sendConfigAck( )
	{
		NODELET_DEBUG( "ARDrone2_Control: Sending config ack" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			boost::mutex::scoped_lock scoped_lock( global_send );
			buf << "AT*CTRL=" << ++sequence << ',' << 5 << ',' << 0 << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
			{
				config_ack.unlock( );
				return true;
			}
		}
		return false;
	}

	bool ARDrone2_Control::sendWatchdogReset( )
	{
		NODELET_DEBUG( "ARDrone2_Control: Sending watchdog reset" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			boost::mutex::scoped_lock scoped_lock( global_send );
			buf << "AT*COMWDG=" << ++sequence << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
			{
				config_ack.unlock( );
				return true;
			}
		}
		return false;
	}

	void ARDrone2_Control::processNavdata( const struct navdata &hdr )
	{
		ros::NodeHandle nh = getNodeHandle( );

		if( hdr.ardrone_state & ARDRONE_NAVDATA_BOOTSTRAP || sequence < 1 )
		{
			NODELET_INFO( "ARDrone2_Control: Handshake complete - Connected" );
			NODELET_DEBUG( "ARDrone2_Control: Drone is in boostrap - sending config" );
			if( !sendConfig( ) )
			{
				NODELET_WARN( "ARDrone2_Control: Failed to send config" );
			}

			NODELET_DEBUG( "ARDrone2_Control: Config has changed, updating cache" );
			boost::thread( &ARDrone2_Control::delayedFetchConfig, this );
		}
		else if( hdr.ardrone_state & ARDRONE_COMMAND_MASK )
		{
			NODELET_DEBUG( "ARDrone2_Control: Drone is acknoledging configuration - sending ack" );
			if( !sendConfigAck( ) )
			{
				NODELET_WARN( "ARDrone2_Control: Failed to send config ack" );
			}
		}
		if( hdr.ardrone_state & ARDRONE_COM_WATCHDOG_MASK )
		{
			NODELET_DEBUG_THROTTLE( 2, "ARDrone2_Control: Communication Watchdog Expired" );
			sendWatchdogReset( );
		}

		if( !twist_sub )
			twist_sub = nh.subscribe( "cmd_vel", 1, &ARDrone2_Control::TwistCB, this );
		if( !imu_pub )
			imu_pub = nh.advertise<sensor_msgs::Imu>( "imu/data", 1 );
		if( !sonar_pub )
			sonar_pub = nh.advertise<sensor_msgs::Range>( "height", 1 );
		if( !twist_pub )
			twist_pub = nh.advertise<geometry_msgs::TwistStamped>( "vel", 1 );
		if( !takeoff_ser )
			takeoff_ser = nh.advertiseService( "takeoff", &ARDrone2_Control::TakeoffCB, this );
		if( !land_ser )
			land_ser = nh.advertiseService( "land", &ARDrone2_Control::LandCB, this );
		if( !trim_ser )
			trim_ser = nh.advertiseService( "flat_trim", &ARDrone2_Control::TrimCB, this );
		if( !reset_ser )
			reset_ser = nh.advertiseService( "reset", &ARDrone2_Control::ResetCB, this );
		if( !cfg_ser )
			cfg_ser = nh.advertiseService( "req_cfg", &ARDrone2_Control::requestConfig, this );
		if( !ledanim_ser )
			ledanim_ser = nh.advertiseService( "led_animate", &ARDrone2_Control::LEDAnimCB, this );
		if( !anim_ser )
			anim_ser = nh.advertiseService( "animate", &ARDrone2_Control::AnimCB, this );
		if( !set_cam_ser )
			set_cam_ser = nh.advertiseService( "set_cam", &ARDrone2_Control::SetCamCB, this );

		last_hdr = hdr;
	}

	void ARDrone2_Control::processNavdataOptions( const std::vector<struct navdata_option> &opts )
	{
		sensor_msgs::ImuPtr imu_msg;

		for( unsigned int i = 0; i < opts.size( ) - 1; i++ )
		{
			switch( opts[i].tag )
			{
				case NAVDATA_DEMO_TAG:
					{
						{
							if( !imu_msg )
							{
								imu_msg = sensor_msgs::ImuPtr( new sensor_msgs::Imu );
								imu_msg->header.stamp = ros::Time::now( );
								imu_msg->header.frame_id = "imu";
							}

							imu_msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(
								*(float *)&opts[i].data.demo_payload.phi / 1000.0 * M_PI / 180,
								-*(float *)&opts[i].data.demo_payload.theta / 1000.0 * M_PI / 180,
								-*(float *)&opts[i].data.demo_payload.psi / 1000.0 * M_PI / 180);
						}
						{
							sensor_msgs::RangePtr msg( new sensor_msgs::Range );

							msg->header.stamp = ros::Time::now( );
							msg->header.frame_id = "sonar";

							msg->radiation_type = sensor_msgs::Range::ULTRASOUND;
							msg->min_range = .220;
							msg->max_range = 6.0;
							msg->range = opts[i].data.demo_payload.altitude / 1000.0;

							sonar_pub.publish( msg );
						}
						{
							geometry_msgs::TwistStampedPtr msg( new geometry_msgs::TwistStamped );

							msg->header.stamp = ros::Time::now( );
							msg->header.frame_id = "base_link";

							msg->twist.linear.x = opts[i].data.demo_payload.vx / 1000.0;
							msg->twist.linear.y = opts[i].data.demo_payload.vy / 1000.0;
							msg->twist.linear.z = opts[i].data.demo_payload.vz / 1000.0;

							twist_pub.publish( msg );
						}
						{
							//ROS_INFO_THROTTLE( 2, "Battery Percent: %u%%", opts[i].data.demo_payload.vbat_flying_percentage );
						}
					}
					break;
				case NAVDATA_RAW_MEASURES_TAG:
					{
						if( have_config )
						{
							if( !imu_msg )
							{
								imu_msg = sensor_msgs::ImuPtr( new sensor_msgs::Imu );
								imu_msg->header.stamp = ros::Time::now( );
								imu_msg->header.frame_id = "imu";
							}

							imu_msg->angular_velocity.x = ( opts[i].data.raw_measures_payload.raw_gyros[0] + gyros_offset[0] ) * gyros_gains[0];
							imu_msg->angular_velocity.y = ( opts[i].data.raw_measures_payload.raw_gyros[1] + gyros_offset[1] ) * -gyros_gains[1];
							imu_msg->angular_velocity.z = ( opts[i].data.raw_measures_payload.raw_gyros[2] + gyros_offset[2] ) * -gyros_gains[2];

							//const float tmp_accel_x = ( opts[i].data.raw_measures_payload.raw_accs[0] - accs_offset[0] );
							//const float tmp_accel_y = ( opts[i].data.raw_measures_payload.raw_accs[1] - accs_offset[1] );
							//const float tmp_accel_z = ( opts[i].data.raw_measures_payload.raw_accs[2] - accs_offset[2] );
							const float tmp_accel_x = opts[i].data.raw_measures_payload.raw_accs[0];
							const float tmp_accel_y = opts[i].data.raw_measures_payload.raw_accs[1];
							const float tmp_accel_z = opts[i].data.raw_measures_payload.raw_accs[2];
							//imu_msg->linear_acceleration.x = accs_gains[0][0] * tmp_accel_x + accs_gains[1][0] * tmp_accel_y + accs_gains[2][0] * tmp_accel_z + accs_offset[0];
							//imu_msg->linear_acceleration.y = accs_gains[0][1] * tmp_accel_x + accs_gains[1][1] * tmp_accel_y + accs_gains[2][1] * tmp_accel_z + accs_offset[1];
							//imu_msg->linear_acceleration.z = accs_gains[0][2] * tmp_accel_x + accs_gains[1][2] * tmp_accel_y + accs_gains[2][2] * tmp_accel_z + accs_offset[2];
							imu_msg->linear_acceleration.x = ( accs_gains[0][0] * tmp_accel_x + accs_gains[0][1] * tmp_accel_y + accs_gains[0][2] * tmp_accel_z + accs_offset[0] ) / 98.8;
							imu_msg->linear_acceleration.y = ( accs_gains[1][0] * tmp_accel_x + accs_gains[1][1] * tmp_accel_y + accs_gains[1][2] * tmp_accel_z + accs_offset[1] ) / -98.8;
							imu_msg->linear_acceleration.z = ( accs_gains[2][0] * tmp_accel_x + accs_gains[2][1] * tmp_accel_y + accs_gains[2][2] * tmp_accel_z + accs_offset[2] ) / -98.8;
							//imu_msg->linear_acceleration.x = tmp_accel_x;
							//imu_msg->linear_acceleration.y = tmp_accel_y;
							//imu_msg->linear_acceleration.z = tmp_accel_z;

						}

					}
					{
						//ROS_INFO_THROTTLE( 2, "Battery Voltage: %2.3fv", opts[i].data.raw_measures_payload.vbat_raw / 1000.0 );
					}
					break;
				//case NAVDATA_PHYS_MEASURES_TAG:
				//	break;
				case NAVDATA_VISION_DETECT_TAG:
					break;
				default:
					NODELET_WARN_THROTTLE( 10, "ARDrone2_Control: Unknown navdata option #%d", opts[i].tag );
					break;
			}
		}

		if( imu_msg )
			imu_pub.publish( imu_msg );
	}

	void ARDrone2_Control::TwistCB( const geometry_msgs::TwistPtr &msg )
	{
		if( sockfd_out != -1 && last_hdr.ardrone_state & ARDRONE_FLY_MASK )
		{
			NODELET_DEBUG( "ARDrone2_Control: Sending control" );
			std::stringstream buf;
			float roll = -msg->linear.y;
			float pitch = -msg->linear.x;
			float yaw = -msg->angular.z;
			float gaz = msg->linear.z;

			boost::mutex::scoped_lock scoped_lock( global_send );
			buf << "AT*PCMD=" << ++sequence << ',' << 1 << ',' << *(int *)&roll << ',' << *(int *)&pitch << ',' << *(int *)&gaz << ',' << *(int *)&yaw << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) != (signed)buf.str( ).length( ) )
				NODELET_WARN( "ARDrone2_Control: Failed to send control" );
		}
	}

	bool ARDrone2_Control::TakeoffCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		NODELET_INFO( "ARDrone2_Control: Sending takeoff" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			boost::mutex::scoped_lock scoped_lock( global_send );
			buf << "AT*REF=" << ++sequence << ',' << ( 290717696 | ( 1 << 9 ) ) << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
				return true;
		}
		return false;
	}

	bool ARDrone2_Control::LandCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		NODELET_INFO( "ARDrone2_Control: Sending land" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			boost::mutex::scoped_lock scoped_lock( global_send );
			buf << "AT*REF=" << ++sequence << ',' << 290717696 << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
				return true;
		}
		return false;
	}

	bool ARDrone2_Control::TrimCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		NODELET_INFO( "ARDrone2_Control: Sending Flat Trim" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			boost::mutex::scoped_lock scoped_lock( global_send );
			buf << "AT*FTRIM=" << ++sequence << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
				return true;
		}
		return false;
	}

	bool ARDrone2_Control::ResetCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & )
	{
		NODELET_INFO( "ARDrone2_Control: Sending Emergency Toggle" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			boost::mutex::scoped_lock scoped_lock( global_send );
			buf << "AT*REF=" << ++sequence << ',' << ( 290717696 | ( 1 << 8 ) ) << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
				return true;
		}
		return false;
	}

	bool ARDrone2_Control::LEDAnimCB( smd_ardrone2::DroneLEDAnimate::Request &msg, smd_ardrone2::DroneLEDAnimate::Response & )
 	{
		NODELET_DEBUG( "ARDrone2_Control: Sending LED Animation Config" );
 		std::stringstream buf;
 		if( sockfd_out != -1 )
 		{
			global_config.lock( );
			global_send.lock( );
			buf << "AT*CONFIG=" << ++sequence << ",\"leds:leds_anim\",\"" << msg.num << ',' << *(int *)&msg.freq << ',' << msg.dur << '\"' << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
			{
				global_send.unlock( );
				config_ack.lock( );
				config_ack.lock( );
				config_ack.unlock( );
				global_config.unlock( );

				return true;
			}
			else
			{
				global_send.unlock( );
				global_config.unlock( );
			}
		}
		return false;
	}

	bool ARDrone2_Control::AnimCB( smd_ardrone2::DroneAnimate::Request &msg, smd_ardrone2::DroneAnimate::Response & )
	{
		NODELET_DEBUG( "ARDrone2_Control: Sending Animation Config" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			if( msg.dur < 0 && msg.num < sizeof(MAYDAY_TIMEOUT) / sizeof( MAYDAY_TIMEOUT[0] ) )
				msg.dur = MAYDAY_TIMEOUT[msg.num];
			global_config.lock( );
			global_send.lock( );
			buf << "AT*CONFIG=" << ++sequence << ",\"control:flight_anim\",\"" << msg.num << ',' << msg.dur << '\"' << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
			{
				global_send.unlock( );
				config_ack.lock( );
				config_ack.lock( );
				config_ack.unlock( );
				global_config.unlock( );

				return true;
			}
			else
			{
				global_send.unlock( );
				global_config.unlock( );
			}
		}
		return false;
	}

	bool ARDrone2_Control::SetCamCB( smd_ardrone2::DroneSetCam::Request &msg, smd_ardrone2::DroneSetCam::Response & )
	{
		NODELET_DEBUG( "ARDrone2_Control: Sending Camera Config" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			global_config.lock( );
			global_send.lock( );
			buf << "AT*CONFIG=" << ++sequence << ",\"video:video_channel\",\"" << (unsigned int)msg.num << '\"' << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
			{
				global_send.unlock( );
				config_ack.lock( );
				config_ack.lock( );
				config_ack.unlock( );
				global_config.unlock( );

				return true;
			}
			else
			{
				global_send.unlock( );
				global_config.unlock( );
			}
		}
		return false;
	}

	bool ARDrone2_Control::requestConfig( smd_ardrone2::DroneConfig::Request &, smd_ardrone2::DroneConfig::Response &ret_msg )
	{
		return fetchConfig( ret_msg.cfg );
	}

	void ARDrone2_Control::delayedFetchConfig( )
	{
		std::string tmp;
		boost::this_thread::sleep( boost::posix_time::seconds( 2 ) );
		for( unsigned char i = 0; i < 5 && fetchConfig( tmp ); i++ );
	}

	bool ARDrone2_Control::fetchConfig( std::string &configuration )
	{
		NODELET_DEBUG( "ARDrone2_Control: Connecting to configuration port" );

		int sockfd_cfg;
		int ret;
		struct addrinfo hints, *servinfo = NULL, *p = NULL;
		struct timeval tv;

		memset( &hints, 0, sizeof hints );
		hints.ai_family = AF_INET;		// IPv4 Only (for now)
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;

		tv.tv_sec   = (int)sockto;
		tv.tv_usec  = ( sockto - tv.tv_sec ) * 1000000;

		if( ( ret = getaddrinfo( sockaddr.c_str( ), "5559", &hints, &servinfo ) ) != 0 )
		{
			NODELET_DEBUG( "ARDrone2_Control: Failed to connect to config: %s", gai_strerror( ret ) );
			return false;
		}

		for( p = servinfo; p != NULL; p = p->ai_next )
		{
			if( ( sockfd_cfg = socket( p->ai_family, p->ai_socktype, p->ai_protocol ) ) == -1 )
			{
				NODELET_DEBUG( "ARDrone2_Control: Skipping invalid socket" );
				continue;
			}

			if( setsockopt( sockfd_cfg, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( tv ) ) )
			{
				NODELET_DEBUG( "ARDrone2_Control: Failed to set socket receive timeout" );
				close( sockfd_cfg );
				sockfd_cfg = -1;
				continue;
			}

			if( setsockopt( sockfd_cfg, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof( tv ) ) )
			{
				NODELET_DEBUG( "ARDrone2_Control: Failed to set socket send timeout" );
				close( sockfd_cfg );
				sockfd_cfg = -1;
				continue;
			}

			if ( ::connect( sockfd_cfg, p->ai_addr, p->ai_addrlen ) == -1 )
			{
				NODELET_DEBUG( "ARDrone2_Control: Connection attempt failed" );
				close( sockfd_cfg );
				sockfd_cfg = -1;
				continue;
			}

			break;
		}

		freeaddrinfo( servinfo );

		if( p == NULL )
		{
			NODELET_ERROR( "ARDrone2_Control: Config connection failure" );
			close( sockfd_cfg );
			sockfd_cfg = -1;
			return false;
		}

		NODELET_DEBUG( "ARDrone2_Control: Sending config request" );
		std::stringstream buf;
		if( sockfd_out != -1 )
		{
			global_config.lock( );
			global_send.lock( );
			buf << "AT*CTRL=" << ++sequence << ',' << 4 << ',' << 0 << '\r';
			if( send( sockfd_out, buf.str( ).c_str( ), buf.str( ).length( ), 0 ) == (signed)buf.str( ).length( ) )
			{
				global_send.unlock( );
				config_ack.lock( );
				config_ack.lock( );
				config_ack.unlock( );

				int bytes;
				char cfg[32767];
				char *curr_cfg = cfg;
				while( ( bytes = recv( sockfd_cfg, curr_cfg, sizeof( cfg ) - ( curr_cfg - cfg ), 0 ) ) > 0 )
				{
					curr_cfg += bytes;
				}
				if( curr_cfg == cfg )
				{
					NODELET_WARN( "ARDrone2_Control: Timeout receiving configuration" );
					close( sockfd_cfg );
					global_config.unlock( );
					return false;
				}
				configuration = cfg;

				global_config.unlock( );

				close( sockfd_cfg );

				return parseConfig( configuration );
			}
			else
			{
				global_send.unlock( );
				global_config.unlock( );
			}
		}

		close( sockfd_cfg );

		return false;
	}

	bool ARDrone2_Control::parseConfig( const std::string &configuration )
	{
		std::stringstream conf( configuration );
		std::string key;
		std::string val;

		while( getline( conf, key, '=' ) && key.length( ) && getline( conf, val ) )
		{
			key.erase( key.find_last_not_of( " \n\r\t" ) + 1 );
			val.erase( 0, val.find_first_not_of( " \n\r\t" ) );
			if( key == "control:accs_offset" )
				sscanf( val.c_str( ), " {%f%f%f}", &accs_offset[0], &accs_offset[1], &accs_offset[2] );
			else if( key == "control:accs_gains" )
				sscanf( val.c_str( ), " {%f%f%f%f%f%f%f%f%f}", &accs_gains[0][0], &accs_gains[0][1],
					&accs_gains[0][2], &accs_gains[1][0], &accs_gains[1][1], &accs_gains[1][2],
					&accs_gains[2][0], &accs_gains[2][1], &accs_gains[2][2] );
			else if( key == "control:gyros_offset" )
				sscanf( val.c_str( ), " {%f%f%f}", &gyros_offset[0], &gyros_offset[1], &gyros_offset[2] );
			else if( key == "control:gyros_gains" )
				sscanf( val.c_str( ), " {%f%f%f}", &gyros_gains[0], &gyros_gains[1], &gyros_gains[2] );
		}

		have_config = true;

		return true;
	}
}
