/* C-image processing with numpy arrays
 * base on
 *   http://wiki.scipy.org/Cookbook/C_Extensions/NumPy_arrays  */

#include "Python.h"
#include "numpy/arrayobject.h"
#include <math.h>

/* #### Globals #################################### */

static PyObject *green(PyObject *self, PyObject *args);
static PyObject *avoidGreen(PyObject *self, PyObject *args);

/* .... C vector utility functions ..................*/
int  not_image(PyArrayObject *vec);


/* ==== Set up the methods table ====================== */
static PyMethodDef cimgMethods[] = {
	{"green", green, METH_VARARGS},
	{"avoidGreen", avoidGreen, METH_VARARGS},
	{NULL, NULL}     /* Sentinel - marks the end of this structure */
};

/* ==== Initialize the C_test functions ====================== */
// Module name must be _C_arraytest in compile and linked 
void initcimg()  {
	(void) Py_InitModule("cimg", cimgMethods);
	import_array();  // Must be present for NumPy.  Called first after above line.
}

static PyObject *green(PyObject *self, PyObject *args)
{
	PyArrayObject *vecio;  // The python objects to be extracted from the args
	unsigned char *ptr;             // The C vectors to be created to point to the 
	                                //   python vectors, cin and cout point to the row
	                                //   of vecin and vecout, respectively
	int i,n;
	double frac;
  unsigned char r,g,b;
  int count = 0;
	
	/* Parse tuples separately since args will differ between C fcns */
	if (!PyArg_ParseTuple(args, "O!d", &PyArray_Type, &vecio, &frac))  return NULL;
	if (NULL == vecio)  return NULL;
	
	if (not_image(vecio)) return NULL;
	
	ptr = (unsigned char*)vecio->data;
	
	/* Get vector dimension. */
	n = vecio->dimensions[0] * vecio->dimensions[1];
	
	/* Operate on the vectors  */
	for( i = 0; i < n; i++ )
  {
    r = ptr[2];
    g = ptr[1];
    b = ptr[0];
    if( g > frac * r && g > frac * b )
    {
      ptr[0] = 0;
      ptr[1] = 0xFF;
      ptr[2] = 0;
      count++;
    }
    ptr += 3;
	}
		
	return Py_BuildValue("i", count);
}


static PyObject *avoidGreen(PyObject *self, PyObject *args)
{
	PyArrayObject *vecio;  // The python objects to be extracted from the args
	unsigned char *ptr;             // The C vectors to be created to point to the 
	                                //   python vectors, cin and cout point to the row
	                                //   of vecin and vecout, respectively
	int fromX, toX, fromY, toY, limit;
	double frac;
  int x, y;
  unsigned char r,g,b;
  int count = 0;
	
	/* Parse tuples separately since args will differ between C fcns */
	if (!PyArg_ParseTuple(args, "O!iiiiid", &PyArray_Type, &vecio, &fromX, &toX, &fromY, &toY, &limit, &frac))  return NULL;
	if (NULL == vecio)  return NULL;
	
	if (not_image(vecio)) return NULL;
	
	/* Get vector dimension. */
	n = vecio->dimensions[0] * vecio->dimensions[1];

  if( fromX < toX )
  {
	  for( x = fromX; x < toX; x++ )
    {
      for( y = fromY; y < toY; y++ )
      {
        ptr = (unsigned char*)vecio->data + 3*(x + y*vecio->dimensions[1]);
        r = ptr[2];
        g = ptr[1];
        b = ptr[0];
        if( g > frac * r && g > frac * b )
        {
          ptr[0] = 0;
          ptr[1] = 0xFF;
          ptr[2] = 0;
          count++;
        }
        else
        {
          ptr[0] = 0xFF;
          ptr[1] = 0xFF;
          ptr[2] = 0xFF;
        }
      }
      if( count > limit )
        break;
	  }
  }
  else
  {
	  for( x = fromX; x > toX; x-- )
    {
      for( y = fromY; y < toY; y++ )
      {
        ptr = (unsigned char*)vecio->data + 3*(x + y*vecio->dimensions[1]);
        r = ptr[2];
        g = ptr[1];
        b = ptr[0];
        if( g > frac * r && g > frac * b )
        {
          ptr[0] = 0;
          ptr[1] = 0xFF;
          ptr[2] = 0;
          count++;
        }
        else
        {
          ptr[0] = 0xFF;
          ptr[1] = 0xFF;
          ptr[2] = 0xFF;
        }
      }
      if( count > limit )
        break;
	  }
  }
		
	return Py_BuildValue("i", x);
}


int  not_image(PyArrayObject *vec)  {
	if (vec->descr->type_num != NPY_UINT8 || vec->nd != 3)  {
		PyErr_SetString(PyExc_ValueError,
			"In not_image: array must be of type uint8 and 3 dimensional (n).");
		return 1;  }
	return 0;
}

