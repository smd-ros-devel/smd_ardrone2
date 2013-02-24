#ifndef _ARDrone2_H264_hpp
#define _ARDrone2_H264_hpp

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

#include <boost/thread.hpp>

extern "C"
{ 
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

namespace smd_ardrone2
{
	enum parrot_video_encapsulation_codecs
	{
		CODEC_UNKNNOWN = 0,
		CODEC_VLIB,
		CODEC_P264,
		CODEC_MPEG4_VISUAL,
		CODEC_MPEG4_AVC
	};

	enum parrot_video_encapsulation_frametypes
	{
		FRAME_TYPE_UNKNNOWN = 0,
		FRAME_TYPE_IDR_FRAME,	/* headers followed by I-frame */
		FRAME_TYPE_I_FRAME,
		FRAME_TYPE_P_FRAME,
		FRAME_TYPE_HEADERS
	};

	enum parrot_video_encapsulation_control
	{
		PAVE_CTRL_FRAME_DATA = 0,			/* The PaVE is followed by video data */
		PAVE_CTRL_FRAME_ADVERTISEMENT = ( 1 << 0 ),	/* The PaVE is not followed by any data. Used to announce a frame which will be sent on the other socket later. */
		PAVE_CTRL_LAST_FRAME_IN_STREAM = ( 1 << 1 ),	/* Announces the position of the last frame in the current stream */
	};

	enum parrot_video_encapsulation_stream_id_suffixes
	{
		PAVE_STREAM_ID_SUFFIX_MP4_360p = 0,
		PAVE_STREAM_ID_SUFFIX_H264_360p = 1,
		PAVE_STREAM_ID_SUFFIX_H264_720p = 2
	};

	struct parrot_video_encapsulation
	{
		uint8_t signature[4];
		/* "PaVE" - used to identify the start of
		 * frame */
		uint8_t version;
		/* Version code */
		uint8_t video_codec;
		/* Codec of the following frame */
		uint16_t header_size;
		/* Size of the parrot_video_encapsulation_t
		 * */
		uint32_t payload_size;
		/* Amount of data following this PaVE */
		uint16_t encoded_stream_width;
		/* ex: 640 */
		uint16_t encoded_stream_height;
		/* ex: 368 */
		uint16_t display_width;
		/* ex: 640 */
		uint16_t display_height;
		/* ex: 360 */
		uint32_t frame_number;
		/* Frame position inside the current stream
		 * */
		uint32_t timestamp;
		/* In milliseconds */
		uint8_t total_chuncks;
		/* Number of UDP packets containing the
		 * current decodable payload - currently unused */
		uint8_t chunck_index ;
		/* Position of the packet - first chunk is #0
		 * - currenty unused*/
		uint8_t frame_type;
		/* I-frame, P-frame -
		 * parrot_video_encapsulation_frametypes_t */
		uint8_t control;
		/* Special commands like end-of-stream or
		 * advertised frames */
		uint32_t stream_byte_position_lw; /* Byte position of the current payload in
		the encoded stream - lower 32-bit word */
		uint32_t stream_byte_position_uw; /* Byte position of the current payload in
		the encoded stream - upper 32-bit word */
		uint16_t stream_id;
		/* This ID indentifies packets that should be
		 * recorded together */
		uint8_t total_slices;
		/* number of slices composing the current
		 * frame */
		uint8_t slice_index ;
		/* position of the current slice in the frame
		 * */
		uint8_t header1_size;
		/* H.264 only : size of SPS inside payload -
		 * no SPS present if value is zero */
		uint8_t header2_size;
		/* H.264 only : size of PPS inside payload -
		 * no PPS present if value is zero */
		uint8_t reserved2[2];
		/* Padding to align on 48 bytes */
		uint32_t advertised_size;
		/* Size of frames announced as advertised
		 * frames */
		uint8_t reserved3[12];
		/* Padding to align on 64 bytes */
		uint8_t reserved4[4];
		/* Additional undocumented 4 bytes used
		 * in newer API */
	} __attribute__ ((packed));

	class ARDrone2_H264 : public nodelet::Nodelet
	{
	public:
		ARDrone2_H264( );
		~ARDrone2_H264( );
	private:
		virtual void onInit( );

		bool connect( );
		void disconnect( );
		void spinOnce( );
		void spin( );
		void dumpHeader( const struct parrot_video_encapsulation *hdr ) const;
		void chkUpdateHdr( const struct parrot_video_encapsulation *hdr );
		void processFrame( const struct parrot_video_encapsulation *hdr, const char *data );

		int sockfd;
		std::string sockaddr;
		double sockto;
		double sockcool;
		struct parrot_video_encapsulation last_hdr;

		boost::thread spin_thread;

		AVCodec *h264_codec;
		AVCodecContext *h264_codec_context;
		struct SwsContext *convert_context;
		AVFrame *picture_yuv;
		AVFrame *picture_rgb;

		image_transport::ImageTransport *it;
		image_transport::Publisher image_pub;
	};
}

#endif /* _ARDrone2_H264_hpp */
