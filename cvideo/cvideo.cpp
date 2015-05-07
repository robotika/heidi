#include "Python.h"
#include "numpy/arrayobject.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/mem.h>
}

int  not_image(PyArrayObject *vec)  {
  if (vec->descr->type_num != NPY_UINT8 || vec->nd != 3)  {
    PyErr_SetString(PyExc_ValueError,
      "In not_image: array must be of type uint8 and 3 dimensional (n).");
    return 1;  }
  return 0;
}


// GLOBALS FOR NOW
AVCodec *codecH264, *codecMPEG4;
AVCodecContext *cH264, *cMPEG4;
int got_picture;
AVFrame *picture;
AVFrame *pictureRGB;
AVPacket avpkt;
struct SwsContext *img_convert_ctx;
int srcX,srcY,dstX,dstY; 
uint8_t*bufferBGR;

static PyObject *init(PyObject *self, PyObject *args)
{
//  if (!PyArg_ParseTuple(args, "i", &g_myInt))  return NULL;

  avcodec_register_all();
  av_init_packet(&avpkt);

  codecH264 = avcodec_find_decoder(CODEC_ID_H264);
  codecMPEG4 = avcodec_find_decoder(CODEC_ID_MPEG4);

  if (!codecH264 || !codecMPEG4) 
  {
    fprintf(stderr, "codec not found\n");
    return Py_BuildValue( "i", 0 );
  }
  cH264 = avcodec_alloc_context3(codecH264);
  cMPEG4 = avcodec_alloc_context3(codecMPEG4);
  picture = avcodec_alloc_frame();
  pictureRGB = avcodec_alloc_frame();

  srcX=dstX = 640;
  srcY=dstY = 368;

  img_convert_ctx = sws_getContext(srcX, srcY,PIX_FMT_YUV420P, dstX, dstY, PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);
  if (avcodec_open2(cH264, codecH264, NULL) < 0) {
    fprintf(stderr, "could not open codec H264\n");
    return Py_BuildValue( "i", 0 );
  }
  if (avcodec_open2(cMPEG4, codecMPEG4, NULL) < 0) {
    fprintf(stderr, "could not open codec MPEG\n");
    return Py_BuildValue( "i", 0 );
  }

  bufferBGR = (uint8_t*)av_mallocz(avpicture_get_size(PIX_FMT_BGR24, dstX, dstY) * sizeof(uint8_t)*10);
  avpicture_fill((AVPicture *)pictureRGB,bufferBGR,PIX_FMT_RGB24, dstX, dstY); 

  return Py_BuildValue( "i", 1 );
}

static PyObject *frame(PyObject *self, PyObject *args)
{
  PyArrayObject *numpyImg;
  int width, height;
  int type;

  uint8_t* buf = 0;
  int len;
  if (!PyArg_ParseTuple(args, "O!is#", &PyArray_Type, &numpyImg, &type, &buf, &len))  return NULL;
  if (NULL == numpyImg)  return NULL; 
  if (not_image(numpyImg)) return NULL;

  height = numpyImg->dimensions[0];
  width = numpyImg->dimensions[1];

  if( width != srcX || height != srcY )
  {
    srcX = dstX = width;
    srcY = dstY = height;

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

//  if( pave.video_codec == 4 )
    cSelected = cH264;
  //else
  //  cSelected = cMPEG4;
  avcodec_decode_video2(cSelected, picture, &got_picture, &avpkt);

  if (got_picture > 0)
  {
    sws_scale(img_convert_ctx,picture->data, picture->linesize, 0, srcY-8,pictureRGB->data,pictureRGB->linesize);
    memcpy(numpyImg->data, pictureRGB->data[0], cSelected->width * ((cSelected->height == 368) ? 360 : cSelected->height) * sizeof(uint8_t) * 3);
    return Py_BuildValue( "i", 1 );
  }
  return Py_BuildValue( "i", 0 );
}


static PyMethodDef cvideoMethods[] = {
  {"init", init, METH_VARARGS},
  {"frame", frame, METH_VARARGS},
  {NULL, NULL}     /* Sentinel - marks the end of this structure */
};

extern "C" void initcvideo()
{
  (void) Py_InitModule("cvideo", cvideoMethods);
  import_array();  // Must be present for NumPy.  Called first after above line.
}
