/* C-image processing with numpy arrays
 * base on
 *   http://wiki.scipy.org/Cookbook/C_Extensions/NumPy_arrays  */

#include "Python.h"
#include "numpy/arrayobject.h"
#include <math.h>

/* #### Globals #################################### */

static PyObject *green(PyObject *self, PyObject *args);

/* .... C vector utility functions ..................*/
PyArrayObject *pyvector(PyObject *objin);
double *pyvector_to_Carrayptrs(PyArrayObject *arrayin);
int  not_doublevector(PyArrayObject *vec);


/* ==== Set up the methods table ====================== */
static PyMethodDef cimgMethods[] = {
	{"green", green, METH_VARARGS},
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
	PyArrayObject *vecin, *vecout;  // The python objects to be extracted from the args
	double *cin, *cout;             // The C vectors to be created to point to the 
	                                //   python vectors, cin and cout point to the row
	                                //   of vecin and vecout, respectively
	int i,n;
	const char *str;
	double dfac;
	
	/* Parse tuples separately since args will differ between C fcns */
	if (!PyArg_ParseTuple(args, "O!O!sd", &PyArray_Type, &vecin,
		&PyArray_Type, &vecout, &str, &dfac))  return NULL;
	if (NULL == vecin)  return NULL;
	if (NULL == vecout)  return NULL;
	
	// Print out input string
	printf("Input string: %s\n", str);
	
	/* Check that objects are 'double' type and vectors
	     Not needed if python wrapper function checks before call to this routine */
	if (not_doublevector(vecin)) return NULL;
	if (not_doublevector(vecout)) return NULL;
	
	/* Change contiguous arrays into C * arrays   */
	cin=pyvector_to_Carrayptrs(vecin);
	cout=pyvector_to_Carrayptrs(vecout);
	
	/* Get vector dimension. */
	n=vecin->dimensions[0];
	
	/* Operate on the vectors  */
	for ( i=0; i<n; i++)  {
			cout[i]=2.0*dfac*cin[i];
	}
		
	return Py_BuildValue("i", 1);
}


/* ==== Create 1D Carray from PyArray ======================
    Assumes PyArray is contiguous in memory.             */
double *pyvector_to_Carrayptrs(PyArrayObject *arrayin)  {
	int n;
	
	n=arrayin->dimensions[0];
	return (double *) arrayin->data;  /* pointer to arrayin data as double */
}
/* ==== Check that PyArrayObject is a double (Float) type and a vector ==============
    return 1 if an error and raise exception */ 
int  not_doublevector(PyArrayObject *vec)  {
	if (vec->descr->type_num != NPY_DOUBLE || vec->nd != 1)  {
		PyErr_SetString(PyExc_ValueError,
			"In not_doublevector: array must be of type Float and 1 dimensional (n).");
		return 1;  }
	return 0;
}

