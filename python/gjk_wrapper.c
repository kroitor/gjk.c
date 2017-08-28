/**
Python wrapper of Igor Kroitor implementation Gilbert-Johnson-Keerthi (GJK) collision detection algorithm

Wrapper written for Python 3.5, but it should work with 3.4 and 3.6 too (and maybe future versions).

Author: Gabriel Dubé
*/

#include <Python.h>
#include <gjk.c>

static vec2* sequenceToVec2(PyObject * sequence, Py_ssize_t * sequence_length) {
    const Py_ssize_t length = PySequence_Length(sequence);
    vec2 *vec_sequence = calloc(length, sizeof(vec2));
    PyObject *item, *x, *y;

    for (Py_ssize_t i=0; i<length; i++) {
        item = PySequence_GetItem(sequence, i);
        if (!PySequence_Check(item) || PySequence_Length(item) != 2) {
            Py_DECREF(item);
            PyErr_SetString(PyExc_TypeError, "Arguments items must be list like elements of 2 floats");
            free(vec_sequence);
            return NULL;
        }

        x = PySequence_GetItem(item, 0);
        y = PySequence_GetItem(item, 1);
        vec_sequence[i].x = (float)PyFloat_AsDouble(x);
        vec_sequence[i].y = (float)PyFloat_AsDouble(y);

        Py_DECREF(x);
        Py_DECREF(y);
        Py_DECREF(item);

        if (PyErr_Occurred()) {
            free(vec_sequence);
            return NULL;
        }
    }

    *sequence_length = length;
    return vec_sequence;
}

static PyObject* Gjk_gjk(PyObject * self, PyObject * args) {
    PyObject *_vertices1, *_vertices2, *result;
    vec2 *vertices1 = NULL, *vertices2 = NULL;
    Py_ssize_t count1 = 0, count2 = 0;

    if (!PyArg_ParseTuple(args, "OO", &_vertices1, &_vertices2))
        return NULL;

    if ( !(PySequence_Check(_vertices1) && PySequence_Check(_vertices2)) ) {
        PyErr_SetString(PyExc_TypeError, "Both arguments must be list like objects");
        return NULL;
    }

    vertices1 = sequenceToVec2(_vertices1, &count1);
    vertices2 = sequenceToVec2(_vertices2, &count2);
    if (PyErr_Occurred()) {
        if(vertices1 && !vertices2) free(vertices1);
        return NULL;
    }

    result = gjk(vertices1, count1, vertices2, count2) == 1 ? Py_True : Py_False;
    Py_INCREF(result);
    
    free(vertices1);
    free(vertices2);

    return result;
}

/*
    Gjk functions table
*/
static PyMethodDef Gjk_Methods[] = {
    {"gjk", (PyCFunction)Gjk_gjk, METH_VARARGS, ""},
    {NULL, NULL}
};

/*
    Module definition
*/
static PyModuleDef gjk_mod_mdef = 
{
	PyModuleDef_HEAD_INIT,
	"gjk",
	"Python wrapper of Igor Kroitor implementation Gilbert-Johnson-Keerthi (GJK) collision detection algorithm",
	-1,
	Gjk_Methods
};

PyMODINIT_FUNC PyInit_gjk(void)
{
	PyObject* gjk_mod;
	gjk_mod = PyModule_Create(&gjk_mod_mdef);
	return gjk_mod;
}
