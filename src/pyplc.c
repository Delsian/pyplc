/*
Copyright (c) 2020 Eug Krashtan

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#define PY_SSIZE_T_CLEAN

#include <fcntl.h>
#include "pyplc.h"


PyDoc_STRVAR(PyPlcModuleDoc,
	"This module defines an object type that allows ATPL360 transactions.\n"
	"\n"
	"Because the SPI device interface is opened R/W, users of this\n"
	"module usually must have root permissions.\n");

static PyObject *
PyPlc_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    printf("New\n");
	PyPlcObject *self;
	if ((self = (PyPlcObject *)type->tp_alloc(type, 0)) == NULL)
		return NULL;

	self->fd = -1;
	self->mode = 0;
	self->bits_per_word = 0;
	self->max_speed_hz = 0;

	Py_INCREF(self);
	return (PyObject *)self;
}

PyDoc_STRVAR(PyPlcCloseDoc,
	"close()\n\n"
	"Disconnects the object from the interface.\n");

static PyObject *
PyPlc_close(PyPlcObject *self)
{
	if ((self->fd != -1) && (close(self->fd) == -1)) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	self->fd = -1;
	self->mode = 0;
	self->bits_per_word = 0;
	self->max_speed_hz = 0;

	Py_INCREF(Py_None);
	return Py_None;
}

static void
PyPlc_dealloc(PyPlcObject *self)
{
	PyObject *ref = PyPlc_close(self);
	Py_XDECREF(ref);

	Py_TYPE(self)->tp_free((PyObject *)self);
}

static char *wrmsg_list0 = "Empty argument list.";
static char *wrmsg_listmax = "Argument list size exceeds %d bytes.";
static char *wrmsg_val = "Non-Int/Long value in arguments: %x.";

PyDoc_STRVAR(PyPlcOpenDoc,
	"open(bus, device)\n\n"
	"Connects the object to the specified PLC device.\n"
	"open(X,Y) will open PLC at bus /dev/spidev<X>.<Y>\n");

static PyObject *
PyPlc_open(PyPlcObject *self, PyObject *args, PyObject *kwds)
{
    printf("Open\n");
	int bus, device;
	char path[PYPLC_MAX_BLOCK_SIZE];
	uint8_t tmp8;
	uint32_t tmp32;
	static char *kwlist[] = {"bus", "device", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, kwds, "ii:open", kwlist, &bus, &device))
		return NULL;
	if (snprintf(path, PYPLC_MAX_BLOCK_SIZE, 
            "/dev/spidev%d.%d", bus, device) >= PYPLC_MAX_BLOCK_SIZE) {
		PyErr_SetString(PyExc_OverflowError,
			"Bus and/or device number is invalid.");
		return NULL;
	}
	if ((self->fd = open(path, O_RDWR, 0)) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	if (ioctl(self->fd, SPI_IOC_RD_MODE, &tmp8) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->mode = tmp8;
	if (ioctl(self->fd, SPI_IOC_RD_BITS_PER_WORD, &tmp8) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->bits_per_word = tmp8;
	if (ioctl(self->fd, SPI_IOC_RD_MAX_SPEED_HZ, &tmp32) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->max_speed_hz = tmp32;

    // GPIOs
	for(int i=0; i<GPIO_INDEX_MAX; i++) {
		if(self->pin[i].num < 0) {
			PyErr_SetString(PyExc_TypeError,
            	"Aux pins not configured");
        	return NULL;
		}
	}

	gpio_setup(GPIO_LDO, GPIO_DIR_OUTPUT);
	gpio_setup(GPIO_CS, GPIO_DIR_OUTPUT);
	gpio_setup(GPIO_RST, GPIO_DIR_OUTPUT);
	gpio_setup(GPIO_IRQ, GPIO_DIR_INPUT);

    int err = pl360_init(self);
	if (err < 0) {
		printf("err %d\n", err);
		PyErr_SetString(PyExc_RuntimeError, "PLC init failed");
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

PyDoc_STRVAR(PyPlcRxCbDoc,
	"setrxcb(gpio, cb=None) -> None\n\n"
	"Set callback function to call on packet Rx\n"
	"gpio - IRQ pin number\n");

static PyObject *
PyPlc_setrxcb(PyPlcObject *self, PyObject *args, PyObject *kwargs)
{
	int gpio;
	PyObject *cb_func;
	char *kwlist[] = {"gpio", "callback", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "iO|i", kwlist, &gpio, &cb_func))
		return NULL;

	if (cb_func != NULL && !PyCallable_Check(cb_func))
	{
		PyErr_SetString(PyExc_TypeError, "Parameter must be callable");
		return NULL;
	}

	self->rxcb = cb_func;
	self->pin[GPIO_INDEX_IRQ].num = gpio;
	gpio_setup(GPIO_IRQ, GPIO_DIR_INPUT);

	Py_INCREF(Py_None);
	return Py_None;
}

PyDoc_STRVAR(PyPlcTxDoc,
	"tx(bytearray) -> None\n\n"
	"Write bytes to PLC device.\n");

static PyObject *
PyPlc_tx(PyPlcObject *self, PyObject *args)
{
	uint16_t	ii, len;
	uint8_t	buf[PYPLC_MAX_BLOCK_SIZE];
	PyObject	*obj;
	PyObject	*seq;
	char	wrmsg_text[4096];

	if (!PyArg_ParseTuple(args, "O:write", &obj))
		return NULL;

    seq = PySequence_Fast(obj, "expected a sequence");
	len = PySequence_Fast_GET_SIZE(seq);
	if (!seq || len <= 0) {
		PyErr_SetString(PyExc_TypeError, wrmsg_list0);
		return NULL;
	}

	if (len > PYPLC_MAX_BLOCK_SIZE) {
		snprintf(wrmsg_text, sizeof (wrmsg_text) - 1, wrmsg_listmax, PYPLC_MAX_BLOCK_SIZE);
		PyErr_SetString(PyExc_OverflowError, wrmsg_text);
		return NULL;
	}

	for (ii = 0; ii < len; ii++) {
		PyObject *val = PySequence_Fast_GET_ITEM(seq, ii);
        if (PyLong_Check(val)) {
            buf[ii] = (__u8)PyLong_AS_LONG(val);
        } else {
            snprintf(wrmsg_text, sizeof (wrmsg_text) - 1, wrmsg_val, val);
            PyErr_SetString(PyExc_TypeError, wrmsg_text);
            return NULL;
        }
	}

	Py_DECREF(seq);

    pl360_tx(self, &buf[0], len);

    Py_INCREF(Py_None);
	return Py_None;
}

static int
PyPlc_init(PyPlcObject *self, PyObject *args, PyObject *kwds)
{
    printf("Init\n");
	int bus = -1;
	int client = -1;
	static char *kwlist[] = {"bus", "client", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "|ii:__init__",
			kwlist, &bus, &client))
		return -1;

    gpios_init(self);

	if (bus >= 0) {
		PyPlc_open(self, args, kwds);
		if (PyErr_Occurred())
			return -1;
	}

	return 0;
}

PyDoc_STRVAR(PyPlcObjectType_doc,
	"PyPlc([bus],[client]) -> PLC\n\n"
	"Return a new PLC object that is (optionally) connected to the\n"
	"specified SPI device interface.\n");

static
PyObject *PyPlc_enter(PyObject *self, PyObject *args)
{
    printf("Enter\n");
    if (!PyArg_ParseTuple(args, ""))
        return NULL;

    Py_INCREF(self);
    return self;
}

static
PyObject *PyPlc_exit(PyPlcObject *self, PyObject *args)
{

    PyObject *exc_type = 0;
    PyObject *exc_value = 0;
    PyObject *traceback = 0;
    if (!PyArg_UnpackTuple(args, "__exit__", 3, 3, &exc_type, &exc_value,
                           &traceback)) {
        return 0;
    }

	pl360_stop(self);
    PyPlc_close(self);
    gpios_cleanup(self);
    Py_RETURN_FALSE;
}

static PyObject *
PyPlc_get_pin(PyPlcObject *self, int idx, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->pin[idx].num);
	Py_INCREF(result);
	return result;
}

static int
PyPlc_set_pin(PyPlcObject *self, int idx, PyObject *val, void *closure)
{
   int gpio;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Attribute error");
		return -1;
	}
    if (PyLong_Check(val)) {
        gpio = PyLong_AS_LONG(val);
    } else {
        PyErr_SetString(PyExc_TypeError,
            "The pin number attribute must be an integer");
        return -2;
    }

    self->pin[idx].num = gpio;
    return 0;
}

static PyObject *
PyPlc_get_pin_ldo(PyPlcObject *self, void *closure)
{
	return PyPlc_get_pin(self, GPIO_INDEX_LDO, closure);
}

static int
PyPlc_set_pin_ldo(PyPlcObject *self, PyObject *val, void *closure)
{
    return PyPlc_set_pin(self, GPIO_INDEX_LDO, val, closure);
}

static PyObject *
PyPlc_get_pin_rst(PyPlcObject *self, void *closure)
{
	return PyPlc_get_pin(self, GPIO_INDEX_RST, closure);
}

static int
PyPlc_set_pin_rst(PyPlcObject *self, PyObject *val, void *closure)
{
	return PyPlc_set_pin(self, GPIO_INDEX_RST, val, closure);
}

static PyObject *
PyPlc_get_pin_cs(PyPlcObject *self, void *closure)
{
	return PyPlc_get_pin(self, GPIO_INDEX_CS, closure);
}

static int
PyPlc_set_pin_cs(PyPlcObject *self, PyObject *val, void *closure)
{
    return PyPlc_set_pin(self, GPIO_INDEX_CS, val, closure);
}

static PyObject *
PyPlc_get_pin_irq(PyPlcObject *self, void *closure)
{
	return PyPlc_get_pin(self, GPIO_INDEX_IRQ, closure);
}

static int
PyPlc_set_pin_irq(PyPlcObject *self, PyObject *val, void *closure)
{
    return PyPlc_set_pin(self, GPIO_INDEX_IRQ, val, closure);
}

extern struct spi_ioc_transfer xfer;
static PyObject *
PyPlc_get_speed(PyPlcObject *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", xfer.speed_hz);
	Py_INCREF(result);
	return result;
}

static int
PyPlc_set_speed(PyPlcObject *self, PyObject *val, void *closure)
{
    int speed;

    if (val == NULL) {
        PyErr_SetString(PyExc_TypeError,
            "Attribute error");
        return -1;
    }
    if (PyLong_Check(val)) {
        speed = PyLong_AS_LONG(val);
    } else {
        PyErr_SetString(PyExc_TypeError,
            "The speed attribute must be an integer");
        return -2;
    }

    xfer.speed_hz = speed;
    return 0;
}

static PyGetSetDef PyPlc_getset[] = {
    {"cs", (getter)PyPlc_get_pin_cs, (setter)PyPlc_set_pin_cs,
			"CS pin number\n"},
	{"ldo", (getter)PyPlc_get_pin_ldo, (setter)PyPlc_set_pin_ldo,
			"LDO pin number\n"},
    {"rst", (getter)PyPlc_get_pin_rst, (setter)PyPlc_set_pin_rst,
			"RST pin number\n"},
	{"irq", (getter)PyPlc_get_pin_irq, (setter)PyPlc_set_pin_irq,
			"IRQ pin number\n"},
    {"speed", (getter)PyPlc_get_speed, (setter)PyPlc_set_speed,
			"SPI speed\n"},
	{NULL},
};

static PyMethodDef PyPlcMethods[] = {
	{"open", (PyCFunction)PyPlc_open, METH_VARARGS | METH_KEYWORDS,
		PyPlcOpenDoc},
	{"close", (PyCFunction)PyPlc_close, METH_NOARGS,
		PyPlcCloseDoc},
    {"tx",  (PyCFunction)PyPlc_tx, METH_VARARGS, 
        PyPlcTxDoc},
	{"setrxcb",  (PyCFunction)PyPlc_setrxcb, METH_VARARGS, 
        PyPlcRxCbDoc},
	{"__enter__", (PyCFunction)PyPlc_enter, METH_VARARGS,
		NULL},
	{"__exit__", (PyCFunction)PyPlc_exit, METH_VARARGS,
		NULL},
	{NULL},
};

static PyTypeObject PyPlcObjectType = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"PyPlc",			/* tp_name */
	sizeof(PyPlcObject),		/* tp_basicsize */
	0,				/* tp_itemsize */
	(destructor)PyPlc_dealloc,	/* tp_dealloc */
	0,				/* tp_print */
	0,				/* tp_getattr */
	0,				/* tp_setattr */
	0,				/* tp_compare */
	0,				/* tp_repr */
	0,				/* tp_as_number */
	0,				/* tp_as_sequence */
	0,				/* tp_as_mapping */
	0,				/* tp_hash */
	0,				/* tp_call */
	0,				/* tp_str */
	0,				/* tp_getattro */
	0,				/* tp_setattro */
	0,				/* tp_as_buffer */
	Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags */
	PyPlcObjectType_doc,		/* tp_doc */
	0,				/* tp_traverse */
	0,				/* tp_clear */
	0,				/* tp_richcompare */
	0,				/* tp_weaklistoffset */
	0,				/* tp_iter */
	0,				/* tp_iternext */
	PyPlcMethods,	/* tp_methods */
	0,				/* tp_members */
	PyPlc_getset,   /* tp_getset */
	0,				/* tp_base */
	0,				/* tp_dict */
	0,				/* tp_descr_get */
	0,				/* tp_descr_set */
	0,				/* tp_dictoffset */
	(initproc)PyPlc_init,		/* tp_init */
	0,				/* tp_alloc */
	PyPlc_new,			/* tp_new */
};

static PyMethodDef PyPlcModuleMethods[] = {
	{NULL}
};

static struct PyModuleDef pyplcmodule = {
    PyModuleDef_HEAD_INIT,
    "PyPlc",   /* name of module */
    PyPlcModuleDoc, 
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    PyPlcModuleMethods
};

// Init
PyMODINIT_FUNC
PyInit_pyplc(void)
{
    PyObject *m;

	if (PyType_Ready(&PyPlcObjectType) < 0)
        return NULL;

    // Init module PyPlc
    m = PyModule_Create(&pyplcmodule);
    PyObject *version = PyUnicode_FromString(_VERSION_);

	PyObject *dict = PyModule_GetDict(m);
	PyDict_SetItemString(dict, "__version__", version);
	Py_DECREF(version);

	Py_INCREF(&PyPlcObjectType);
	PyModule_AddObject(m, "PyPlc", (PyObject *)&PyPlcObjectType);

    return m;
}
