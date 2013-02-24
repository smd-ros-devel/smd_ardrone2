#include "smd_ardrone2/ARDrone2_Control.hpp"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>

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
		sockcool( 0.5 )
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
		priv_nh.param( "sock_timeout", sockto, .5 );
		priv_nh.param( "sock_cooldown", sockcool, .5 );

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
			NODELET_DEBUG( "ARDrone2_Control: Failed to connect: %s", gai_strerror( ret ) );
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

		if ( p == NULL )
		{
			NODELET_ERROR( "ARDrone2_Control: Connection failure" );
			return false;
		}

		if( ( ret = getaddrinfo( sockaddr.c_str( ), "5556", &hints, &servinfo ) ) != 0 )
		{
			NODELET_DEBUG( "ARDrone2_Control: Failed to connect: %s", gai_strerror( ret ) );
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

		if ( p == NULL )
		{
			NODELET_ERROR( "ARDrone2_Control: Connection failure" );
			disconnect( );
			return false;
		}

		if( !sendWake( ) )
		{
			NODELET_ERROR( "ARDrone2_Control: Failed to send wake" );
			disconnect( );
			return false;
		}

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
				NODELET_INFO( "ARDrone2_Control: Cooling off for %fs", sockcool );
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
			sequence = 0;
			boost::mutex::scoped_lock scoped_lock( global_send );
			buf << "AT*CONFIG=" << ++sequence << ",\"general:navdata_demo\",\"FALSE\"" << '\r';
			buf << "AT*CONFIG=" << ++sequence << ",\"general:navdata_options\",\"" << 0 << "\"" << '\r';
			buf << "AT*CONFIG=" << ++sequence << ",\"video:video_codec\",\"H264_360P_CODEC\"" << '\r';
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
				return true;
		}
		return false;
	}

	void ARDrone2_Control::processNavdata( const struct navdata &hdr )
	{
		ros::NodeHandle nh = getNodeHandle( );

		if( hdr.ardrone_state & ARDRONE_COMMAND_MASK )
		{
			NODELET_DEBUG( "ARDrone2_Control: Drone has exited - sending ack" );
			if( !sendConfigAck( ) )
			{
				NODELET_WARN( "ARDrone2_Control: Failed to send config ack" );
			}
		}
		else if( hdr.ardrone_state & ARDRONE_NAVDATA_BOOTSTRAP )
		{
			NODELET_DEBUG( "ARDrone2_Control: Drone is in boostrap - sending config" );
			if( !sendConfig( ) )
			{
				NODELET_WARN( "ARDrone2_Control: Failed to send config" );
			}
		}

		if( !twist_sub )
			twist_sub = nh.subscribe( "cmd_vel", 1, &ARDrone2_Control::TwistCB, this );
		if( !imu_pub )
			imu_pub = nh.advertise<sensor_msgs::Imu>( "imu/data", 1 );
		if( !sonar_pub )
			sonar_pub = nh.advertise<sensor_msgs::Range>( "height", 1 );
		if( !twist_pub )
			twist_pub = nh.advertise<geometry_msgs::Twist>( "vel", 1 );
		if( !takeoff_ser )
			takeoff_ser = nh.advertiseService( "takeoff", &ARDrone2_Control::TakeoffCB, this );
		if( !land_ser )
			land_ser = nh.advertiseService( "land", &ARDrone2_Control::LandCB, this );
		if( !trim_ser )
			trim_ser = nh.advertiseService( "flat_trim", &ARDrone2_Control::TrimCB, this );

		last_hdr = hdr;
	}

	void ARDrone2_Control::processNavdataOptions( const std::vector<struct navdata_option> &opts )
	{
		for( unsigned int i = 0; i < opts.size( ) - 1; i++ )
		{
			switch( opts[i].tag )
			{
				case NAVDATA_DEMO_TAG:
					{
						{
							sensor_msgs::ImuPtr msg( new sensor_msgs::Imu );

							msg->header.stamp = ros::Time::now( );
							msg->header.frame_id = "imu";

							msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(
								*(float *)&opts[i].data.demo_payload.phi / 1000.0 * M_PI / 180,
								-*(float *)&opts[i].data.demo_payload.theta / 1000.0 * M_PI / 180,
								-*(float *)&opts[i].data.demo_payload.psi / 1000.0 * M_PI / 180);
							imu_pub.publish( msg );
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
							geometry_msgs::TwistPtr msg( new geometry_msgs::Twist );

							//msg->header.stamp = ros::Time::now( );
							//msg->header.frame_id = "imu";

							msg->linear.x = opts[i].data.demo_payload.vx / 1000.0;
							msg->linear.y = opts[i].data.demo_payload.vy / 1000.0;
							msg->linear.z = opts[i].data.demo_payload.vz / 1000.0;

							twist_pub.publish( msg );
						}
					}
					break;
				case NAVDATA_RAW_MEASURES_TAG:
					break;
				case NAVDATA_PHYS_MEASURES_TAG:
					break;
				case NAVDATA_VISION_DETECT_TAG:
					break;
				default:
					NODELET_WARN( "ARDrone2_Control: Unknown navdata option #%d", opts[i].tag );
					break;
			}
		}
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
}
