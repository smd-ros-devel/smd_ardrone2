#include "smd_ardrone2/ARDrone2_H264.hpp"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>

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

PLUGINLIB_DECLARE_CLASS( smd_ardrone2, ARDrone2_H264, smd_ardrone2::ARDrone2_H264, nodelet::Nodelet )

namespace smd_ardrone2
{
	ARDrone2_H264::ARDrone2_H264( ) :
		sockfd( -1 ),
		sockaddr( "192.168.1.1" ),
		sockto( 0.5 ),
		sockcool( 0.5 ),
		convert_context( NULL ),
		it( NULL )
	{
		memset( &last_hdr, 0, sizeof( last_hdr ) );

		av_register_all( );

		h264_codec = avcodec_find_decoder( CODEC_ID_H264 );
		if( !h264_codec )
		{
			NODELET_ERROR( "ARDrone2_H264: Failed to register codec!" );
			return;
		}

		h264_codec_context = avcodec_alloc_context3( h264_codec );
		if( !h264_codec_context )
		{
			NODELET_ERROR( "ARDrone2_H264: Failed to allocate codec context!" );
			return;
		}

		if( avcodec_get_context_defaults3( h264_codec_context, h264_codec ) )
		{
			NODELET_ERROR( "ARDrone2_H264: Failed set default codec context!" );
			return;
		}

		h264_codec_context->pix_fmt = PIX_FMT_YUV420P;
		h264_codec_context->skip_frame = AVDISCARD_DEFAULT;
		h264_codec_context->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
		h264_codec_context->skip_loop_filter = AVDISCARD_DEFAULT;
		h264_codec_context->workaround_bugs = FF_BUG_AUTODETECT;
		h264_codec_context->codec_type = AVMEDIA_TYPE_VIDEO;
		h264_codec_context->codec_id = CODEC_ID_H264;
		h264_codec_context->skip_idct = AVDISCARD_DEFAULT;

		if( avcodec_open2( h264_codec_context, h264_codec, NULL ) )
		{
			NODELET_ERROR( "ARDrone2_H264: Failed to open codec!" );
			return;
		}

		if( !( picture_yuv = avcodec_alloc_frame( ) ) )
		{
			NODELET_ERROR( "ARDrone2_H264: Failed to allocate YUV frame!" );
			return;
		}

		if( !( picture_rgb = avcodec_alloc_frame( ) ) )
		{
			NODELET_ERROR( "ARDrone2_H264: Failed to allocate RGB frame!" );
			return;
		}
	}

	ARDrone2_H264::~ARDrone2_H264( )
	{
		disconnect( );
		NODELET_DEBUG( "ARDrone2_H264: Killing threads..." );
		spin_thread.interrupt( );
		if( spin_thread.joinable( ) )
			spin_thread.join( );
		avcodec_close( h264_codec_context );
		av_free( h264_codec_context );
		av_free( picture_yuv );
		av_free( picture_rgb );
		sws_freeContext( convert_context );
	}

	void ARDrone2_H264::onInit( )
	{
		ros::NodeHandle nh = getNodeHandle( );
		ros::NodeHandle priv_nh = getPrivateNodeHandle( );

		it = new image_transport::ImageTransport( nh );

		priv_nh.param( "ardrone_addr", sockaddr, (std::string)"192.168.1.1" );
		priv_nh.param( "sock_timeout", sockto, .5 );
		priv_nh.param( "sock_cooldown", sockcool, .5 );

		spin_thread = boost::thread( &ARDrone2_H264::spin, this );
	}

	bool ARDrone2_H264::connect( )
	{
		int ret;
		struct addrinfo hints, *servinfo = NULL, *p = NULL;
		struct timeval tv;

		disconnect( );

		memset( &hints, 0, sizeof hints );
		hints.ai_family = AF_INET;		// IPv4 Only (for now)
		hints.ai_socktype = SOCK_STREAM;

		tv.tv_sec   = (int)sockto;
		tv.tv_usec  = ( sockto - tv.tv_sec ) * 1000000;

		if( ( ret = getaddrinfo( sockaddr.c_str( ), "5555", &hints, &servinfo ) ) != 0 )
		{
			NODELET_DEBUG( "ARDrone2_H264: Failed to connect: %s", gai_strerror( ret ) );
			return false;
		}

		for( p = servinfo; p != NULL; p = p->ai_next )
		{
			if( ( sockfd = socket( p->ai_family, p->ai_socktype, p->ai_protocol ) ) == -1 )
			{
				NODELET_DEBUG( "ARDrone2_H264: Skipping invalid socket" );
				continue;
			}

			if( setsockopt( sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( tv ) ) )
			{
				NODELET_DEBUG( "ARDrone2_H264: Failed to set socket receive timeout" );
				close( sockfd );
				sockfd = -1;
				continue;
			}

			if( setsockopt( sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof( tv ) ) )
			{
				NODELET_DEBUG( "ARDrone2_H264: Failed to set socket send timeout" );
				close( sockfd );
				sockfd = -1;
				continue;
			}

			if ( ::connect( sockfd, p->ai_addr, p->ai_addrlen ) == -1 )
			{
				NODELET_DEBUG( "ARDrone2_H264: Connection attempt failed" );
				close( sockfd );
				sockfd = -1;
				continue;
			}

			break;
		}

		freeaddrinfo( servinfo );

		if ( p == NULL )
		{
			NODELET_ERROR( "ARDrone2_H264: Connection failure" );
			return false;
		}

		NODELET_INFO( "ARDrone2_H264: Connected" );

		NODELET_DEBUG( "ARDrone2_H264: Advertising..." );
		image_pub = it->advertise( "image_raw", 1 );

		return true;
	}

	void ARDrone2_H264::disconnect( )
	{
		close( sockfd );
		sockfd = -1;
		image_pub.shutdown( );
	}

	void ARDrone2_H264::spinOnce( )
	{
		static struct parrot_video_encapsulation hdr;
		static int bytes;

		if( sockfd == -1 )
		{
			if( !connect( ) )
			{
				NODELET_INFO( "ARDrone2_H264: Cooling off for %fs", sockcool );
				usleep( sockcool * 1000000 );
				return;
			}
		}

		if( ( bytes = recv( sockfd, &(hdr.signature), sizeof( hdr.signature ), 0 ) ) == -1 )
		{
			NODELET_WARN( "ARDrone2_H264: Timeout receiving data" );
			disconnect( );
			return;
		}

		if( bytes == sizeof( hdr.signature ) && !strncmp( (char *)&hdr.signature, "PaVE", 4 ) )
		{
			int bytes_to_go = sizeof( hdr.version ) + sizeof( hdr.video_codec ) + sizeof( hdr.header_size );
			char *buff = (char *)&hdr + sizeof( hdr.signature );
			char *data;

			while( bytes_to_go )
			{
				if( ( bytes = recv( sockfd, buff, bytes_to_go, 0 ) ) == -1 )
				{
					NODELET_WARN( "ARDrone2_H264: Timeout receiving data" );
					disconnect( );
					return;
				}
				bytes_to_go -= bytes;
				buff += bytes;
			}

			bytes_to_go = hdr.header_size - ( sizeof( hdr.signature ) + sizeof( hdr.version ) + sizeof( hdr.video_codec ) + sizeof( hdr.header_size ) );
			buff = (char *)&hdr + sizeof( hdr.signature ) + sizeof( hdr.version ) + sizeof( hdr.video_codec ) + sizeof( hdr.header_size );

			while( bytes_to_go )
			{
				if( ( bytes = recv( sockfd, buff, bytes_to_go, 0 ) ) == -1 )
				{
					NODELET_WARN( "ARDrone2_H264: Timeout receiving data" );
					disconnect( );
					return;
				}
				bytes_to_go -= bytes;
				buff += bytes;
			}

			data = new char[hdr.payload_size];
			buff = data;
			bytes_to_go = hdr.payload_size;

			if( !bytes_to_go )
			{
				NODELET_DEBUG( "ARDrone2_H264: Received empty frame - ignoring it" );
				delete [] data;
				return;
			}

			while( bytes_to_go )
			{
				if( ( bytes = recv( sockfd, buff, bytes_to_go, 0 ) ) == -1 )
				{
					NODELET_WARN( "ARDrone2_H264: Timeout receiving data" );
					delete [] data;
					disconnect( );
					return;
				}
				bytes_to_go -= bytes;
				buff += bytes;
			}

			chkUpdateHdr( &hdr );
			if( last_hdr.frame_number + 1 == hdr.frame_number || hdr.frame_type == FRAME_TYPE_IDR_FRAME )
				processFrame( &hdr, data );
			else
				NODELET_DEBUG( "ARDrone2_H264: Skipping a frame" );

			delete [] data;
		}
		else
		{
			if( bytes == 0 )
			{
				NODELET_WARN( "ARDrone2_H264: Connected, but receiving no data" );
				disconnect( );
				return;
			}
			char recv_hdr[5] = { 0 };
			memcpy( (char *)&recv_hdr, (char *)&hdr.signature, 4 );
			NODELET_WARN( "ARDrone2_H264: Skipping %d bytes (Bad Signature \"%s\")", bytes, recv_hdr );
		}
	}

	void ARDrone2_H264::spin( )
	{
		while( true )
		{
			boost::this_thread::interruption_point( );
			spinOnce( );
		}
	}

	void ARDrone2_H264::chkUpdateHdr( const struct parrot_video_encapsulation *hdr )
	{
		if( hdr->encoded_stream_width != last_hdr.encoded_stream_width ||
			hdr->encoded_stream_height != last_hdr.encoded_stream_height ||
			hdr->display_width != last_hdr.display_width ||
			hdr->display_height != last_hdr.display_height ||
			hdr->stream_id != last_hdr.stream_id )
		{
			int numBytes;
			uint8_t *buffer;
			NODELET_INFO( "ARDrone2_H264: Stream dimension change detected" );

			h264_codec_context->width = hdr->encoded_stream_width;
			h264_codec_context->height = hdr->encoded_stream_height;

			numBytes = avpicture_get_size( h264_codec_context->pix_fmt, hdr->display_width, hdr->display_height );
			buffer = (uint8_t *)av_realloc( picture_yuv->data[0], numBytes * sizeof( char ) );
			avpicture_fill( (AVPicture *)picture_yuv, buffer, h264_codec_context->pix_fmt, hdr->display_width, hdr->display_height );

			numBytes = avpicture_get_size( PIX_FMT_RGB24, hdr->display_width, hdr->display_height );
			buffer = (uint8_t *)av_realloc( picture_rgb->data[0], numBytes * sizeof( char ) );
			avpicture_fill( (AVPicture *)picture_rgb, buffer, PIX_FMT_RGB24, hdr->display_width, hdr->display_height );

			convert_context = sws_getCachedContext( convert_context, hdr->display_width, hdr->display_height, h264_codec_context->pix_fmt, hdr->display_width, hdr->display_height, PIX_FMT_RGB24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
		}

		memcpy( &last_hdr, hdr, sizeof( hdr ) );
	}

	void ARDrone2_H264::processFrame( const struct parrot_video_encapsulation *hdr, const char *data )
	{
			AVPacket pkt;
			int frameFinished = 0;

			av_init_packet( &pkt );

			pkt.data = (uint8_t *)data;
			pkt.size = hdr->payload_size;

			avcodec_decode_video2( h264_codec_context, picture_yuv, &frameFinished, &pkt );

			if( !frameFinished )
			{
				NODELET_WARN( "ARDrone2_H264: Failed to decode frame #%d", hdr->frame_number );
				return;
			}

			if( !convert_context || !sws_scale( convert_context, picture_yuv->data, picture_yuv->linesize, 0, hdr->display_height, picture_rgb->data, picture_rgb->linesize ) )
			{
				NODELET_WARN( "ARDrone2_H264: Failed to convert frame #%d", hdr->frame_number );
				return;
			}

			sensor_msgs::ImagePtr msg( new sensor_msgs::Image );

			msg->header.stamp = ros::Time::now( );
			msg->header.frame_id = "forward_camera";
			msg->height = picture_yuv->height;
			msg->width = picture_yuv->width;
			msg->encoding = "rgb8";
			msg->is_bigendian = 0;
			msg->step = picture_rgb->linesize[0];
			msg->data.assign( picture_rgb->data[0], picture_rgb->data[0] + 3 * msg->width * msg->height );

			image_pub.publish( msg );

			NODELET_DEBUG( "ARDrone2_H264: Processed frame #%d", hdr->frame_number );

			last_hdr = *hdr;
	}

	void ARDrone2_H264::dumpHeader( const struct parrot_video_encapsulation *hdr ) const
	{
		std::cerr << "ARDrone2_H264: Header" << std::endl
			<< "Version: " << (unsigned int)hdr->version << std::endl
			<< "Video Codec: " << (unsigned int)hdr->video_codec << std::endl
			<< "Header Size: " << (unsigned int)hdr->header_size << std::endl
			<< "Payload Size: " << (unsigned int)hdr->payload_size << std::endl
			<< "Width: " << (unsigned int)hdr->encoded_stream_width << std::endl
			<< "Height: " << (unsigned int)hdr->encoded_stream_height << std::endl
			<< "Display Width: " << (unsigned int)hdr->display_width << std::endl
			<< "Display Height: " << (unsigned int)hdr->display_height << std::endl
			<< "Frame Number: " << (unsigned int)hdr->frame_number << std::endl
			<< "Timestamp: " << (unsigned int)hdr->timestamp << std::endl
			<< "Total Chuncks: " << (unsigned int)hdr->total_chuncks << std::endl
			<< "Chunck Index: " << (unsigned int)hdr->chunck_index << std::endl
			<< "Frame Type: " << (unsigned int)hdr->frame_type << std::endl
			<< "Control: " << (unsigned int)hdr->control << std::endl
			<< "Byte Position (LW): " << (unsigned int)hdr->stream_byte_position_lw << std::endl
			<< "Byte Position (UW): " << (unsigned int)hdr->stream_byte_position_uw << std::endl
			<< "Stream ID: " << (unsigned int)hdr->stream_id << std::endl
			<< "Total Slices: " << (unsigned int)hdr->total_slices << std::endl
			<< "Slice Index: " << (unsigned int)hdr->slice_index << std::endl
			<< "SPS Size: " << (unsigned int)hdr->header1_size << std::endl
			<< "PPS Size: " << (unsigned int)hdr->header2_size << std::endl
			<< "Advertised Size: " << (unsigned int)hdr->advertised_size << std::endl
			<< std::endl;
	}
}
