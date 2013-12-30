import struct
import sys
import gzip

"""
#define NAVDATA_PORT              5554
#define NAVDATA_HEADER            0x55667788
#define NAVDATA_BUFFER_SIZE       2048  

    NAVDATA_DEMO_TAG = 0,
    NAVDATA_VISION_DETECT_TAG = 16,
    NAVDATA_IPHONE_ANGLES_TAG = 18,
    NAVDATA_CKS_TAG = 0xFFFF 


NAVDATA_OPTION( navdata_time_t,           navdata_time            , NAVDATA_TIME_TAG             ) 1
NAVDATA_OPTION( navdata_raw_measures_t,   navdata_raw_measures    , NAVDATA_RAW_MEASURES_TAG     ) 2
NAVDATA_OPTION( navdata_phys_measures_t,  navdata_phys_measures   , NAVDATA_PHYS_MEASURES_TAG    ) 3
NAVDATA_OPTION( navdata_gyros_offsets_t,  navdata_gyros_offsets   , NAVDATA_GYROS_OFFSETS_TAG    ) 4
NAVDATA_OPTION( navdata_euler_angles_t,   navdata_euler_angles    , NAVDATA_EULER_ANGLES_TAG     ) 5
NAVDATA_OPTION( navdata_references_t,     navdata_references      , NAVDATA_REFERENCES_TAG       ) 6
NAVDATA_OPTION( navdata_trims_t,          navdata_trims           , NAVDATA_TRIMS_TAG            ) 7
NAVDATA_OPTION( navdata_rc_references_t,  navdata_rc_references   , NAVDATA_RC_REFERENCES_TAG    ) 8
NAVDATA_OPTION( navdata_pwm_t,            navdata_pwm             , NAVDATA_PWM_TAG              ) 9
NAVDATA_OPTION( navdata_altitude_t,       navdata_altitude        , NAVDATA_ALTITUDE_TAG         ) 10
NAVDATA_OPTION( navdata_vision_raw_t,     navdata_vision_raw      , NAVDATA_VISION_RAW_TAG       ) 11
NAVDATA_OPTION( navdata_vision_of_t,      navdata_vision_of       , NAVDATA_VISION_OF_TAG        ) 12
NAVDATA_OPTION( navdata_vision_t,         navdata_vision          , NAVDATA_VISION_TAG           ) 13
NAVDATA_OPTION( navdata_vision_perf_t ,   navdata_vision_perf     , NAVDATA_VISION_PERF_TAG      ) 14
NAVDATA_OPTION( navdata_trackers_send_t,  navdata_trackers_send   , NAVDATA_TRACKERS_SEND_TAG    ) 15
NAVDATA_OPTION( navdata_vision_detect_t,  navdata_vision_detect   , NAVDATA_VISION_DETECT_TAG    ) 16
NAVDATA_OPTION( navdata_watchdog_t  ,     navdata_watchdog        , NAVDATA_WATCHDOG_TAG         ) 17
NAVDATA_OPTION( navdata_adc_data_frame_t, navdata_adc_data_frame  , NAVDATA_ADC_DATA_FRAME_TAG   ) 18
NAVDATA_OPTION( navdata_video_stream_t,   navdata_video_stream    , NAVDATA_VIDEO_STREAM_TAG     ) 19
NAVDATA_OPTION( navdata_games_t,          navdata_games           , NAVDATA_GAMES_TAG            ) 20
NAVDATA_OPTION( navdata_pressure_raw_t,   navdata_pressure_raw    , NAVDATA_PRESSURE_RAW_TAG		 ) 21
NAVDATA_OPTION( navdata_magneto_t,       	navdata_magneto					, NAVDATA_MAGNETO_TAG					 ) 22
NAVDATA_OPTION( navdata_wind_speed_t,     navdata_wind_speed      , NAVDATA_WIND_TAG						 ) 23
NAVDATA_OPTION( navdata_kalman_pressure_t,navdata_kalman_pressure	, NAVDATA_KALMAN_PRESSURE_TAG	 ) 24
NAVDATA_OPTION( navdata_hdvideo_stream_t ,navdata_hdvideo_stream  , NAVDATA_HDVIDEO_STREAM_TAG	 ) 25
NAVDATA_OPTION( navdata_wifi_t           ,navdata_wifi            , NAVDATA_WIFI_TAG             ) 26
 

typedef struct _navdata_magneto_t {
  uint16_t   tag;
  uint16_t   size;

  int16_t   	mx;
  int16_t   	my;
  int16_t   	mz;
  vector31_t 	magneto_raw;       // magneto in the body frame, in mG
  vector31_t 	magneto_rectified;
  vector31_t 	magneto_offset;
  float32_t 	heading_unwrapped;
  float32_t 	heading_gyro_unwrapped;
  float32_t 	heading_fusion_unwrapped;
  char 			magneto_calibration_ok;
  uint32_t      magneto_state;
  float32_t 	magneto_radius;
  float32_t     error_mean;
  float32_t     error_var;

}_ATTRIBUTE_PACKED_ navdata_magneto_t; 


"""
NAVDATA_DEMO_TAG = 0
NAVDATA_TIME_TAG = 1
NAVDATA_RAW_MEASURES_TAG = 2
NAVDATA_ALTITUDE_TAG = 10
NAVDATA_VISION_DETECT_TAG = 16
NAVDATA_IPHONE_ANGLES_TAG = 18
NAVDATA_MAGNETO_TAG = 22
NAVDATA_CKS_TAG = 0xFFFF

#    drone_state['navdata_bootstrap']    = _[1] >> 11 & 1 # Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */ 
#     drone_state['com_watchdog_mask']    = _[1] >> 30 & 1 # Communication Watchdog : (1) com problem, (0) Com is ok */ 

#    drone_state['command_mask']         = _[1] >>  6 & 1 # Control command ACK : (0) None, (1) one received */ 

def parseTimeTag( data, offset ):
  "uint32_t - the 11 most significant bits represents the seconds, and the 21 least significant bits are the microseconds."
# tag == NAVDATA_TIME_TAG:
  utime = struct.unpack_from("I", data, offset+4)[0]
  assert utime & (1<<20) == 0, str( (hex(utime), header[2]) )
  return (utime >> 21) + (utime & 0xFFFFF)/1000000.0


def parseDemoTag( data, offset ):
  values = struct.unpack_from("IIfffifff", data, offset+4)
  values = dict(zip(['ctrl_state', 'battery', 'theta', 'phi', 'psi', 'altitude', 'vx', 'vy', 'vz'], values))
  values['ctrl_state'] >>= 16
  return values


def parseVisionDetectTag( data, offset ):
  "tag == NAVDATA_VISION_DETECT_TAG"
  countTypes = struct.unpack_from("IIIII", data, offset+4)
  x = struct.unpack_from("IIII", data, offset+4+20)
  y = struct.unpack_from("IIII", data, offset+4+36)
  height = struct.unpack_from("IIII", data, offset+4+52)
  width = struct.unpack_from("IIII", data, offset+4+68)
  dist = struct.unpack_from("IIII", data, offset+4+84)
  oriAngle = struct.unpack_from("ffff", data, offset+4+100)
  # size 328, skip 3x3 matrix + 3x1 matrix = 12*4 = 48 bytes *4 NB_NAVDATA_DETECTION_RESULTS = 192
  cameraSource = struct.unpack_from("IIII", data, offset+4+116+192)
  return (countTypes, x, y, height, width, dist, oriAngle, cameraSource)

def parseRawMeasuresTag( data, offset ):
  "tag == NAVDATA_RAW_MEASURES_TAG"
  # 52 bytes = 2*18+3*4=36+12=48 (+4header)
  raw = struct.unpack_from("=HHHhhhhhIhhhhhhhhhIih", data, offset+4)
  return raw[:3]


def parseAltitudeTag( data, offset ):
  """  tag == NAVDATA_ALTITUDE_TAG
typedef struct _navdata_altitude_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t   altitude_vision;
  float32_t altitude_vz;
  int32_t   altitude_ref;
  int32_t   altitude_raw;

  float32_t		obs_accZ;
  float32_t 	obs_alt;
  vector31_t 	obs_x;
  uint32_t 		obs_state;
  vector21_t	est_vb;
  uint32_t 		est_state ;

}_ATTRIBUTE_PACKED_ navdata_altitude_t; 
"""
#      (altVision, altVz, altRef, altRaw) = 
  return struct.unpack_from("ifiiff", data, offset+4)

def parseMagnetoTag( data, offset ):
  """ tag == NAVDATA_MAGNETO_TAG
typedef struct _navdata_magneto_t {
  uint16_t   tag;
  uint16_t   size;

  int16_t   	mx;
  int16_t   	my;
  int16_t   	mz;
  vector31_t 	magneto_raw;       // magneto in the body frame, in mG
  vector31_t 	magneto_rectified;
  vector31_t 	magneto_offset;
  float32_t 	heading_unwrapped;
  float32_t 	heading_gyro_unwrapped;
  float32_t 	heading_fusion_unwrapped;
  char 			magneto_calibration_ok;
  uint32_t      magneto_state;
  float32_t 	magneto_radius;
  float32_t     error_mean;
  float32_t     error_var;

}_ATTRIBUTE_PACKED_ navdata_magneto_t; 
"""
  return struct.unpack_from("=hhhffffffffffffcIfff", data, offset+4)


def parseNavData( packet ):
  offset = 0
  header =  struct.unpack_from("IIII", packet, offset)
  print "ID", header[2], hex(header[1]), "ACK", header[1] >> 6 & 1, "WDOG", header[1] >> 30 & 1
  time,alt,ctrl = None,None,None
  offset += struct.calcsize("IIII")
  while True:
    tag, size =  struct.unpack_from("HH", packet, offset)
    offset += struct.calcsize("HH")
    values = []
    print tag, size
    for i in range(size-struct.calcsize("HH")):
      values.append(struct.unpack_from("c", packet, offset)[0])
      offset += struct.calcsize("c")
    if tag == NAVDATA_CKS_TAG:
      break
    if tag == NAVDATA_DEMO_TAG:
      values = parseDemoTag( "ABCD" + "".join(values), 0 )
      print values
      print "CTRL STATE", values['ctrl_state']
      alt = values['altitude']
      ctrl = values['ctrl_state']
#CTRL_DEFAULT=0, CTRL_INIT=1, CTRL_LANDED=2, CTRL_FLYING=3, CTRL_HOVERING=4, CTRL_TEST=5, 
#CTRL_TRANS_TAKEOFF=6, CTRL_TRANS_GOTOFIX=7, CTRL_TRANS_LANDING=8, CTRL_TRANS_LOOPING=9

    if tag == NAVDATA_TIME_TAG:
      time = parseTimeTag("ABCD"+"".join(values), 0)
      print "TIME\t%f\t%s" % (time, hex(struct.unpack_from("I", "".join(values))[0]))

    if tag == NAVDATA_RAW_MEASURES_TAG:
      # 52 bytes = 2*18+3*4=36+12=48 (+4header)
      raw = struct.unpack_from("=HHHhhhhhIhhhhhhhhhIih", "".join(values))
      print "RAW", "\t".join( [str(x) for x in raw] )
      print parseRawMeasuresTag( "ABCD"+"".join(values), 0 )

    if tag == NAVDATA_ALTITUDE_TAG:
      (altVision, altVz, altRef, altRaw) = struct.unpack_from("ifii", "".join(values))
#      print "ALT", (altVision, altVz, altRef, altRaw)

    if tag == NAVDATA_MAGNETO_TAG:
      magneto = parseMagnetoTag("ABCD"+ "".join(values), 0 )
      values = dict(zip(['mx', 'my', 'mz', 
        'magneto_raw_x', 'magneto_raw_y', 'magneto_raw_z', 
        'magneto_rectified_x', 'magneto_rectified_y', 'magneto_rectified_z',
        'magneto_offset_x', 'magneto_offset_y', 'magneto_offset_z',
        'heading_unwrapped', 'heading_gyro_unwrapped', 'heading_fusion_unwrapped', 
        'magneto_calibration_ok', 'magneto_state', 'magneto_radius',
        'error_mean', 'error_var'], magneto))
#      print "compass:\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f" % tuple([values[x] for x in ['mx', 'my', 'mz', 
#        'magneto_raw_x', 'magneto_raw_y', 'magneto_raw_z',
#        'magneto_rectified_x', 'magneto_rectified_y', 'magneto_rectified_z',
#        'magneto_offset_x', 'magneto_offset_y', 'magneto_offset_z',
#        ]])
      print "compass:\t"+"\t".join( [str(x) for x in magneto] )

    if tag == NAVDATA_VISION_DETECT_TAG:
      (countTypes, x, y, height, width, dist, oriAngle, cameraSource) = parseVisionDetectTag( "ABCD" + "".join(values), 0 )
      print "TAG", countTypes
      print "TAG", x[0],y[0], height[0], width[0], dist[0]/100.0, oriAngle[0], cameraSource[1]
  if time and ctrl != 2:
    print "ALT\t%f\t%d\t%d" % (time, ctrl, alt)
  return offset


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)

  filename = sys.argv[1]
  if filename.endswith(".gz"):
    packet = gzip.open(filename, "rb").read()
  else:
    packet = open(filename, "rb").read()

  offset = 0
  while offset < len(packet):
  #for i in xrange(3):
    offset += parseNavData( packet[offset:] )
