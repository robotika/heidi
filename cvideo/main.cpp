extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>  // Video write

using namespace cv;


#define INBUF_SIZE 4096 

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

#pragma pack(push,1)
typedef struct {
	uint8_t signature[4]; /* "PaVE" - used to identify the start of
				 frame */
	uint8_t version; /* Version code */
	uint8_t video_codec; /* Codec of the following frame */
	uint16_t header_size; /* Size of the parrot_video_encapsulation_t
				 */
	uint32_t payload_size; /* Amount of data following this PaVE */
	uint16_t encoded_stream_width; /* ex: 640 */
	uint16_t encoded_stream_height; /* ex: 368 */
	uint16_t display_width; /* ex: 640 */
	uint16_t display_height; /* ex: 360 */
	uint32_t frame_number; /* Frame position inside the current stream
				  */
	uint32_t timestamp; /* In milliseconds */
	uint8_t total_chuncks; /* Number of UDP packets containing the
				  current decodable payload - currently unused */
	uint8_t chunck_index ; /* Position of the packet - first chunk is #0
				  - currenty unused*/
	uint8_t frame_type; /* I-frame, P-frame -
			       parrot_video_encapsulation_frametypes_t */
	uint8_t control; /* Special commands like end-of-stream or
			    advertised frames */
	uint32_t stream_byte_position_lw; /* Byte position of the current payload in
					     the encoded stream - lower 32-bit word */
	uint32_t stream_byte_position_uw; /* Byte position of the current payload in
					     the encoded stream - upper 32-bit word */
	uint16_t stream_id; /* This ID indentifies packets that should be
			       recorded together */
	uint8_t total_slices; /* number of slices composing the current
				 frame */
	uint8_t slice_index ; /* position of the current slice in the frame
				 */
	uint8_t header1_size; /* H.264 only : size of SPS inside payload -
				 no SPS present if value is zero */
	uint8_t header2_size; /* H.264 only : size of PPS inside payload -
				 no PPS present if value is zero */
	uint8_t reserved2[2]; /* Padding to align on 48 bytes */
	uint32_t advertised_size; /* Size of frames announced as advertised
				     frames */
	uint8_t reserved3[12]; /* Padding to align on 64 bytes */
  uint32_t md_hack_68;
} /*__attribute__ ((packed))*/ parrot_video_encapsulation_t; 
#pragma pack(pop)

int main(int argc, char **argv)
{
  bool saveVideo = false;
  if( argc < 2 )
  {
    fprintf(stderr, "Missing filename\n");
    exit(-1);
  }
  parrot_video_encapsulation_t pave; 
  int h264MoveIndex = -1;

  AVCodec *codecH264, *codecMPEG4;
  AVCodecContext *cH264, *cMPEG4;
  int got_picture;
  AVFrame *picture;
  AVFrame *pictureRGB;
  AVPacket avpkt;
  struct SwsContext *img_convert_ctx;
  int srcX,srcY,dstX,dstY; 

  avcodec_register_all();
  av_init_packet(&avpkt);

  codecH264 = avcodec_find_decoder(CODEC_ID_H264);
  codecMPEG4 = avcodec_find_decoder(CODEC_ID_MPEG4);

  if (!codecH264 || !codecMPEG4) 
  {
    fprintf(stderr, "codec not found\n");
    exit(1);
  }
  cH264 = avcodec_alloc_context3(codecH264);
  cMPEG4 = avcodec_alloc_context3(codecMPEG4);
  picture = avcodec_alloc_frame();
  pictureRGB = avcodec_alloc_frame();

  FILE *fd = fopen( argv[1], "rb" );
  if( fd == NULL )
    return -1;

  // CODEC
  union { int v; char c[5];} uEx ;
  uEx.c[0] = 'H';
  uEx.c[1] = '2';
  uEx.c[2] = '6';
  uEx.c[3] = '4';
  uEx.c[4]='\0';

  VideoWriter outputVideo;
  if( saveVideo )
  {
    Size S = Size( 640, 360 );
    //  outputVideo.open(NAME , ex, inputVideo.get(CV_CAP_PROP_FPS),S, true);
    const string source      = argv[1];           // the source file name
    string::size_type pAt = source.find_last_of('.');                  // Find extension point
    //  const string NAME = string(argv[1])+string(".avi");
    const string NAME = source.substr(0, pAt) + ".avi";   // Form the new name with container
    //  outputVideo.open( "output.avi" , uEx.v, 30,S, true);
    outputVideo.open( "output.avi" , -1, 30,S, true);
  }

  srcX=dstX = 640;
  srcY=dstY = 368;

  img_convert_ctx = sws_getContext(srcX, srcY,PIX_FMT_YUV420P, dstX, dstY, PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);
  if (avcodec_open2(cH264, codecH264, NULL) < 0) {
    fprintf(stderr, "could not open codec H264\n");
    exit(1);
  }
  if (avcodec_open2(cMPEG4, codecMPEG4, NULL) < 0) {
    fprintf(stderr, "could not open codec MPEG\n");
    exit(1);
  }

  uint8_t*bufferBGR = (uint8_t*)av_mallocz(avpicture_get_size(PIX_FMT_BGR24, dstX, dstY) * sizeof(uint8_t)*10);
  avpicture_fill((AVPicture *)pictureRGB,bufferBGR,PIX_FMT_RGB24, dstX, dstY); 


  IplImage *img;
  img = cvCreateImage(cvSize(640,360), IPL_DEPTH_8U, 3); // c is not set yet!!!
  if (!img)
  {
    return -2;
  }

  // Clear the image
  cvZero(img);

  uint8_t* buf = 0;
  int len;
  int type;

  for(;;)
  {
    len = fread(&pave, 1, sizeof(parrot_video_encapsulation_t), fd); // hack header
    if( len == 0 )
      break;

    buf = (uint8_t*)malloc( pave.payload_size );

    // every firmware has a little bit different header - it is necessary to parse header.size
    if( len < pave.header_size )
      fread(buf, 1, pave.header_size-len, fd );

    len = fread(buf, 1, pave.payload_size, fd);
    if( len < (int)pave.payload_size )
      break;


    type = pave.frame_type;

    if( pave.encoded_stream_width != srcX || pave.encoded_stream_height != srcY )
    {
      srcX = dstX = pave.display_width;
      srcY = dstY = pave.display_height;
      cvReleaseImage( &img );
      img = cvCreateImage(cvSize(srcX,srcY), IPL_DEPTH_8U, 3); // c is not set yet!!!
      if( !img )
        return -2;
      cvZero(img);

      av_freep( bufferBGR );
      bufferBGR = (uint8_t*)av_mallocz(avpicture_get_size(PIX_FMT_BGR24, dstX, dstY) * sizeof(uint8_t)*10);
      avpicture_fill((AVPicture *)pictureRGB,bufferBGR,PIX_FMT_RGB24, dstX, dstY);

      sws_freeContext(img_convert_ctx); 
      img_convert_ctx = sws_getContext(srcX, srcY,PIX_FMT_YUV420P, dstX, dstY, PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);
    }

    avpkt.size = len;
    avpkt.data = buf;
    avpkt.flags = 0;
    if (type == 1) avpkt.flags=AV_PKT_FLAG_KEY;

    AVCodecContext *cSelected;  // not allocated, just temporary pointer

    if( pave.video_codec == 4 )
      cSelected = cH264;
    else
      cSelected = cMPEG4;
    avcodec_decode_video2(cSelected, picture, &got_picture, &avpkt);

    if (got_picture > 0)
    {
      sws_scale(img_convert_ctx,picture->data, picture->linesize, 0, srcY-8,pictureRGB->data,pictureRGB->linesize);
    }
    fprintf( stderr, "TIMESTAMP\t%d\n", pave.timestamp );

    memcpy(img->imageData, pictureRGB->data[0], cSelected->width * ((cSelected->height == 368) ? 360 : cSelected->height) * sizeof(uint8_t) * 3);

    if( h264MoveIndex < 0 && pave.frame_type == 1 )
      h264MoveIndex = 0; // init after first I-frame

    if( h264MoveIndex >=0 && pave.frame_type == 3 )
      h264MoveIndex++;

    if( saveVideo )
      outputVideo.write( img );

    cvShowImage( "camera", img );
//    int key = cvWaitKey(100);
    int key = cvWaitKey(1);
    if( true ) //key == 'p' || key == 's' || (h264MoveIndex % 3 == 2) )
    {
      char namebuf[256];
//      sprintf( namebuf, "img_%d.jpg", pave.timestamp );
      sprintf( namebuf, "img_%04d.jpg", h264MoveIndex );
      cvSaveImage( namebuf, img );
    }
    if( key == 27 || key == 'q' )
      break;
    free(buf);
  }
  fclose(fd);
  cvReleaseImage(&img);

  sws_freeContext(img_convert_ctx); 
  avcodec_close(cH264);
  avcodec_close(cMPEG4);
  av_free(cH264);
  av_free(cMPEG4);
  av_free(picture); 

  return 0;
}
