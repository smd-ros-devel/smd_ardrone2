#ifndef _ARDrone2_Control_hpp
#define _ARDrone2_Control_hpp

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

#include <netinet/in.h>

namespace smd_ardrone2
{
	enum ardrone_state_mask
	{
		ARDRONE_FLY_MASK		= 1U << 0,  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
		ARDRONE_VIDEO_MASK		= 1U << 1,  /*!< VIDEO MASK : (0) video disable, (1) video enable */
		ARDRONE_VISION_MASK		= 2U << 2,  /*!< VISION MASK : (0) vision disable, (1) vision enable */
		ARDRONE_CONTROL_MASK		= 1U << 3,  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
		ARDRONE_ALTITUDE_MASK		= 1U << 4,  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
		ARDRONE_USER_FEEDBACK_START	= 1U << 5,  /*!< USER feedback : Start button state */
		ARDRONE_COMMAND_MASK		= 1U << 6,  /*!< Control command ACK : (0) None, (1) one received */
		ARDRONE_CAMERA_MASK		= 1U << 7,  /*!< CAMERA MASK : (0) camera not ready, (1) Camera ready */
		ARDRONE_TRAVELLING_MASK		= 1U << 8,  /*!< Travelling mask : (0) disable, (1) enable */
		ARDRONE_USB_MASK		= 1U << 9,  /*!< USB key : (0) usb key not ready, (1) usb key ready */
		ARDRONE_NAVDATA_DEMO_MASK	= 1U << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
		ARDRONE_NAVDATA_BOOTSTRAP	= 1U << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
		ARDRONE_MOTORS_MASK		= 1U << 12, /*!< Motors status : (0) Ok, (1) Motors problem */
		ARDRONE_COM_LOST_MASK		= 1U << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
		ARDRONE_SOFTWARE_FAULT		= 1U << 14, /*!< Software fault detected - user should land as quick as possible (1) */
		ARDRONE_VBAT_LOW		= 1U << 15, /*!< VBat low : (1) too low, (0) Ok */
		ARDRONE_USER_EL			= 1U << 16, /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
		ARDRONE_TIMER_ELAPSED		= 1U << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
		ARDRONE_MAGNETO_NEEDS_CALIB	= 1U << 18, /*!< Magnetometer calibration state : (0) Ok, no calibration needed, (1) not ok, calibration needed */
		ARDRONE_ANGLES_OUT_OF_RANGE	= 1U << 19, /*!< Angles : (0) Ok, (1) out of range */
		ARDRONE_WIND_MASK		= 1U << 20, /*!< WIND MASK: (0) ok, (1) Too much wind */
		ARDRONE_ULTRASOUND_MASK		= 1U << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
		ARDRONE_CUTOUT_MASK		= 1U << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
		ARDRONE_PIC_VERSION_MASK	= 1U << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
  		DRONE_ATCODEC_THREAD_ON		= 1U << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
		ARDRONE_NAVDATA_THREAD_ON	= 1U << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
		ARDRONE_VIDEO_THREAD_ON		= 1U << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
		ARDRONE_ACQ_THREAD_ON		= 1U << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
		ARDRONE_CTRL_WATCHDOG_MASK	= 1U << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
		ARDRONE_ADC_WATCHDOG_MASK	= 1U << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
		ARDRONE_COM_WATCHDOG_MASK	= 1U << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
		ARDRONE_EMERGENCY_MASK		= 1U << 31  /*!< Emergency landing : (0) no emergency, (1) emergency */
	};

	enum navdata_options
	{
		NAVDATA_DEMO_TAG = 0,
		NAVDATA_TIME_TAG,
		NAVDATA_RAW_MEASURES_TAG,
		NAVDATA_PHYS_MEASURES_TAG,
		NAVDATA_GYROS_OFFSETS_TAG,
		NAVDATA_EULER_ANGLES_TAG,
		NAVDATA_REFERENCES_TAG,
		NAVDATA_TRIMS_TAG,
		NAVDATA_RC_REFERENCES_TAG,
		NAVDATA_PWM_TAG,
		NAVDATA_ALTITUDE_TAG,
		NAVDATA_VISION_RAW_TAG,
		NAVDATA_VISION_OF_TAG,
		NAVDATA_VISION_TAG,
		NAVDATA_VISION_PERF_TAG,
		NAVDATA_TRACKERS_SEND_TAG,
		NAVDATA_VISION_DETECT_TAG,
		NAVDATA_WATCHDOG_TAG,
		NAVDATA_ADC_DATA_FRAME_TAG,
		NAVDATA_VIDEO_STREAM_TAG,
		NAVDATA_GAMES_TAG,
		NAVDATA_PRESSURE_RAW_TAG,
		NAVDATA_MAGNETO_TAG,
		NAVDATA_WIND_TAG,
		NAVDATA_KALMAN_PRESSURE_TAG,
		NAVDATA_HDVIDEO_STREAM_TAG,
		NAVDATA_WIFI_TAG,
		NAVDATA_ZIMMU_3000_TAG,
		NAVDATA_CKS_TAG = 0xFFFF
	};

	struct navdata
	{
		uint32_t header;           /*!< Always set to NAVDATA_HEADER */
		uint32_t ardrone_state;    /*!< Bit mask built from def_ardrone_state_mask_t */
		uint32_t sequence;         /*!< Sequence number, incremented for each sent packet */
		uint32_t vision_defined;
	} __attribute__ ((packed));

	struct navdata_option
	{
		uint16_t tag;	/*!< Navdata block ('option') identifier */
		uint16_t size;  /*!< set this to the size of this structure */
		union data
		{
			struct navdata_demo
			{
				uint32_t ctrl_state;			/*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
				uint32_t vbat_flying_percentage;	/*!< battery voltage filtered (mV) */

				float theta;				/*!< UAV's pitch in milli-degrees */
				float phi;				/*!< UAV's roll  in milli-degrees */
				float psi;				/*!< UAV's yaw   in milli-degrees */

				int32_t altitude;			/*!< UAV's altitude in centimeters */

				float vx;				/*!< UAV's estimated linear velocity */
				float vy;				/*!< UAV's estimated linear velocity */
				float vz;				/*!< UAV's estimated linear velocity */

				uint32_t num_frames;			/*!< streamed frame index */ // Not used -> To integrate in video stage.

				// Camera parameters compute by detection
				float detection_camera_rot[9];		/*!<  Deprecated ! Don't use ! */
				float detection_camera_trans[3];	/*!<  Deprecated ! Don't use ! */
				uint32_t detection_tag_index;		/*!<  Deprecated ! Don't use ! */

				uint32_t detection_camera_type;		/*!<  Type of tag searched in detection */

				// Camera parameters compute by drone
				float drone_camera_rot[9];		/*!<  Deprecated ! Don't use ! */
				float drone_camera_trans[3];		/*!<  Deprecated ! Don't use ! */
			} __attribute__ ((packed)) demo_payload;

			struct navdata_time
			{
				uint32_t time;	/*!< 32 bit value where the 11 most significant bits represents the seconds, and the 21 least significant bits are the microseconds. */
			} __attribute__ ((packed)) time_payload;

			struct navdata_raw_measures
			{
				// +12 bytes
				uint16_t raw_accs[3];    // filtered accelerometers
				int16_t raw_gyros[3];  // filtered gyrometers
				int16_t raw_gyros_110[2];     // gyrometers  x/y 110 deg/s
				uint32_t vbat_raw;             // battery voltage raw (mV)
				uint16_t us_debut_echo;
				uint16_t us_fin_echo;
				uint16_t us_association_echo;
				uint16_t us_distance_echo;
				uint16_t us_courbe_temps;
				uint16_t us_courbe_valeur;
				uint16_t us_courbe_ref;
				uint16_t flag_echo_ini;
				// TODO: uint16_t  frame_number; // from ARDrone_Magneto
				uint16_t nb_echo;
				uint32_t sum_echo;
				int32_t alt_temp_raw;

				int16_t gradient;
			} __attribute__ ((packed)) raw_measures_payload;

			struct navdata_pressure_raw
			{
				int32_t up;
				int16_t ut;
				int32_t Temperature_meas;
				int32_t Pression_meas;
			} __attribute__ ((packed)) pressure_raw_payload;

			struct navdata_magneto
			{
				int16_t mx;
				int16_t my;
				int16_t mz;
				float magneto_raw[3];       // magneto in the body frame, in mG
				float magneto_rectified[3];
				float magneto_offset[3];
				float heading_unwrapped;
				float heading_gyro_unwrapped;
				float heading_fusion_unwrapped;
				char magneto_calibration_ok;
				uint32_t magneto_state;
				float magneto_radius;
				float error_mean;
				float error_var;
			} __attribute__ ((packed)) magneto_payload;

			struct navdata_wind_speed
			{
				float wind_speed;
				float wind_angle;
				float wind_compensation_theta;
				float wind_compensation_phi;
				float state_x1;
				float state_x2;
				float state_x3;
				float state_x4;
				float state_x5;
				float state_x6;
				float magneto_debug1;
				float magneto_debug2;
				float magneto_debug3;
			} __attribute__ ((packed)) wind_speed_payload;

			struct navdata_kalman_pressure
			{
				float offset_pressure;
				float est_z;
				float est_zdot;
				float est_bias_PWM;
				float est_biais_pression;
				float offset_US;
				float prediction_US;
				float cov_alt;
				float cov_PWM;
				float cov_vitesse;
				uint8_t bool_effet_sol;
				float somme_inno;
				uint8_t flag_rejet_US;
				float u_multisinus;
				float gaz_altitude;
				uint8_t Flag_multisinus;
				uint8_t Flag_multisinus_debut;
			} __attribute__ ((packed)) kalman_pressure_payload;

			struct navdata_zimmu_3000
			{
				int32_t vzimmuLSB;
				float vzfind;
			} __attribute__ ((packed)) zimmu_3000_payload;

			struct navdata_phys_measures
			{
				float accs_temp;
				uint16_t gyro_temp;
				float phys_accs[3];
				float phys_gyros[3];
				uint32_t alim3V3;		// 3.3volt alim [LSB]
				uint32_t vrefEpson;		// ref volt Epson gyro [LSB]
				uint32_t vrefIDG;		// ref volt IDG gyro [LSB]
			} __attribute__ ((packed)) phys_measures_payload;

			struct navdata_gyros_offsets
			{
				float offset_g[3];
			} __attribute__ ((packed)) gyros_offsets_payload;

			struct navdata_euler_angles
			{
				float theta_a;
				float phi_a;
			} __attribute__ ((packed)) euler_angles_payload;

			struct navdata_references
			{
				int32_t ref_theta;
				int32_t ref_phi;
				int32_t ref_theta_I;
				int32_t ref_phi_I;
				int32_t ref_pitch;
				int32_t ref_roll;
				int32_t ref_yaw;
				int32_t ref_psi;

				float vx_ref;
				float vy_ref;
				float theta_mod;
				float phi_mod;

				float k_v_x;
				float k_v_y;
				uint32_t k_mode;

				float ui_time;
				float ui_theta;
				float ui_phi;
				float ui_psi;
				float ui_psi_accuracy;
				int32_t ui_seq;
			} __attribute__ ((packed)) references_payload;

			struct navdata_trims
			{
				float angular_rates_trim_r;
				float euler_angles_trim_theta;
				float euler_angles_trim_phi;
			} __attribute__ ((packed)) trims_payload;

			struct navdata_rc_references
			{
				int32_t rc_ref_pitch;
				int32_t rc_ref_roll;
				int32_t rc_ref_yaw;
				int32_t rc_ref_gaz;
				int32_t rc_ref_ag;
			} __attribute__ ((packed)) rc_references_payload;

			struct navdata_pwm
			{
				uint8_t motor1;
				uint8_t motor2;
				uint8_t motor3;
				uint8_t motor4;
				uint8_t sat_motor1;
				uint8_t sat_motor2;
				uint8_t sat_motor3;
				uint8_t sat_motor4;
				float gaz_feed_forward;
				float gaz_altitude;
				float altitude_integral;
				float vz_ref;
				int32_t u_pitch;
				int32_t u_roll;
				int32_t u_yaw;
				float yaw_u_I;
				int32_t u_pitch_planif;
				int32_t u_roll_planif;
				int32_t u_yaw_planif;
				float u_gaz_planif;
				uint16_t current_motor1;
				uint16_t current_motor2;
				uint16_t current_motor3;
				uint16_t current_motor4;
				//WARNING: new navdata (FC 26/07/2011)
				float altitude_prop;
				float altitude_der;
			} __attribute__ ((packed)) pwm_payload;

			struct navdata_altitude
			{
				int32_t altitude_vision;
				float altitude_vz;
				int32_t altitude_ref;
				int32_t altitude_raw;

				float obs_accZ;
				float obs_alt;
				float obs_x[3];
				uint32_t obs_state;
				float est_vb[2];
				uint32_t est_state ;
			} __attribute__ ((packed)) altitude_payload;

			struct navdata_vision_raw
			{
				float vision_tx_raw;
				float vision_ty_raw;
				float vision_tz_raw;
			} __attribute__ ((packed)) vision_raw_payload;

			struct navdata_vision
			{
				uint32_t vision_state;
				int32_t vision_misc;
				float vision_phi_trim;
				float vision_phi_ref_prop;
				float vision_theta_trim;
				float vision_theta_ref_prop;

				int32_t new_raw_picture;
				float theta_capture;
				float phi_capture;
				float psi_capture;
				int32_t altitude_capture;
				uint32_t time_capture;     // time in TSECDEC format (see config.h)
				float body_v[3];

				float delta_phi;
				float delta_theta;
				float delta_psi;

				uint32_t gold_defined;
				uint32_t gold_reset;
				float gold_x;
				float gold_y;
			} __attribute__ ((packed)) vision_payload;

			struct navdata_vision_perf
			{
				// +44 bytes
				float time_szo;
				float time_corners;
				float time_compute;
				float time_tracking;
				float time_trans;
				float time_update;
				float time_custom[20];
			} __attribute__ ((packed)) vision_perf_payload;

			/*struct navdata_trackers_send
			{
				int32_t locked[DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT];
				screen_point_t point[DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT];
			} __attribute__ ((packed)) trackers_send_payload;*/

			struct navdata_vision_detect
			{
				/* !! Change the function 'navdata_server_reset_vision_detect()' if this structure is modified !! */
				uint32_t nb_detected;
				uint32_t type[4];
				uint32_t xc[4];
				uint32_t yc[4];
				uint32_t width[4];
				uint32_t height[4]; 
				uint32_t dist[4];
				float orientation_angle[4];
				float rotation[4][9];
				float translation[4][3];
				uint32_t camera_source[4];
			} __attribute__ ((packed)) vision_detect_payload;

			struct navdata_vision_of
			{
				float of_dx[5];
				float of_dy[5];
			} __attribute__ ((packed)) vision_of_payload;

			struct navdata_watchdog
			{
				// +4 bytes
				int32_t watchdog;
			} __attribute__ ((packed)) watchdog_payload;

			struct navdata_adc_data_frame
			{
				uint32_t version;
				uint8_t data_frame[32];
			} __attribute ((packed)) adc_data_frame_payload;

			struct navdata_video_stream
			{
				uint8_t quant;			// quantizer reference used to encode frame [1:31]
				uint32_t frame_size;		// frame size (bytes)
				uint32_t frame_number;		// frame index
				uint32_t atcmd_ref_seq;		// atmcd ref sequence number
				uint32_t atcmd_mean_ref_gap;	// mean time between two consecutive atcmd_ref (ms)
				float atcmd_var_ref_gap;
				uint32_t atcmd_ref_quality;	// estimator of atcmd link quality

				// drone2
				uint32_t out_bitrate;		// measured out throughput from the video tcp socket
				uint32_t desired_bitrate;	// last frame size generated by the video encoder

				// misc temporary data
				int32_t data1;
				int32_t data2;
				int32_t data3;
				int32_t data4;
				int32_t data5;

				// queue usage
				uint32_t tcp_queue_level;
				uint32_t fifo_queue_level;
			} __attribute__ ((packed)) video_stream_payload;

			struct navdata_hdvideo_stream
			{
				uint32_t hdvideo_state;
				uint32_t storage_fifo_nb_packets;
				uint32_t storage_fifo_size;
				uint32_t usbkey_size;			/*! USB key in kbytes - 0 if no key present */
				uint32_t usbkey_freespace;		/*! USB key free space in kbytes - 0 if no key present */
				uint32_t frame_number;		/*! 'frame_number' PaVE field of the frame starting to be encoded for the HD stream */
				uint32_t usbkey_remaining_time;	/*! time in seconds */
			} __attribute__ ((packed)) hdvideo_stream_payload;

			struct navdata_games
			{
				uint32_t double_tap_counter;
				uint32_t finish_line_counter;
			} __attribute__ ((packed)) games_payload;

			struct navdata_wifi
			{
				uint32_t link_quality;
			} __attribute__ ((packed)) wifi_payload;

			struct navdata_cks
			{
				uint32_t cks;
			} __attribute__ ((packed)) cks_payload;
		} data;
	} __attribute ((packed));

	class ARDrone2_Control : public nodelet::Nodelet
	{
	public:
		ARDrone2_Control( );
		~ARDrone2_Control( );
	private:
		virtual void onInit( );

		bool connect( );
		void disconnect( );
		void spinOnce( );
		void spin( );
		void TwistCB( const geometry_msgs::TwistPtr &msg );
		bool TakeoffCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & );
		bool LandCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & );
		bool TrimCB( std_srvs::Empty::Request &, std_srvs::Empty::Response & );
		void processNavdata( const struct navdata &hdr );
		void processNavdataOptions( const std::vector<struct navdata_option> &opts );
		bool processChecksum( const struct navdata &hdr, const std::vector<struct navdata_option> &opts ) const;
		bool sendWake( );
		bool sendConfig( );
		bool sendConfigAck( );
		void dumpHeader( const struct navdata *hdr ) const;

		int sockfd_in;
		int sockfd_out;
		std::string sockaddr;
		double sockto;
		double sockcool;
		long unsigned int sequence;
		struct navdata last_hdr;

		boost::thread spin_thread;
		boost::mutex global_send;

		ros::Subscriber twist_sub;
		ros::Publisher imu_pub;
		ros::Publisher sonar_pub;
		ros::Publisher twist_pub;
		ros::ServiceServer takeoff_ser;
		ros::ServiceServer land_ser;
		ros::ServiceServer trim_ser;
	};
}

#endif /* _ARDrone2_Control_hpp */
