#include <iostream>

#include <gtest/gtest.h>
#include <Python.h>

#include "DataStructure/IMUMeasure.h"
#include "IMU/IMU.h"


TEST(imuPropagation, IMU) {

    /// get test data
    Py_Initialize();
    ASSERT_TRUE(Py_IsInitialized());
    PyObject *IMUTest = NULL;
    PyObject *getData = NULL;
    pModule =PyImport_ImportModule("./IMU/test/IMUTest.py");
    ASSERT_TRUE(IMUTest!= NULL);
    getData = PyObject_GetAttrString(IMUTest, "getData");
    ASSERT_TRUE(getData!= NULL);
    PyObject *getDataArgs = PyTuple_New(2);
    PyTuple_SetItem(getDataArgs, 0, Py_BuildValue("s", "./IMU/test/data.csv"));
    PyTuple_SetItem(getDataArgs, 1, Py_BuildValue("i", 100));     ///< get 100 data
    PyObject *pyDatas = NULL;
    pyDatas = PyEval_CallObject(pFunc, pArgs);
    ASSERT_TRUE(pyDatas!= NULL);

    int dataSize = PyList_Size(pyDatas);
    ImuMeasureDeque testIMUDeque;
    PyObject* pyData = NULL;
    PyObject* pyItem = NULL;
    for(int i = 0; i < dataSize; ++i) {
        pyData = PyList_GetItem(pyDatas);
        ASSERT_TRUE(PyList_Size(pyData) == 7);
        double pyArg[7];
        for(int j = 0; j < 7; ++j) {
            PyList_GetItem(pyItem, j);
            PyArg_Parse(pyItem, "d", pyArg + j);
        }
        double timeStamp = pyArg[0];
        Eigen::Vector3d gyroscopes(pyArg[1],pyArg[2],pyArg[3]);
        Eigen::Vector3d acceleration(pyArg[4], pyArg[5], pyArg[6]);
        testIMUDeque.push_back(IMUMeasure(0, timeStamp, acceleration, gyroscopes));
    }

    Py_DECREF(getDataArgs);
    Py_DECREF(IMUTest);
    Py_Finalize();

    ///< over!
    ASSERT_TRUE(testIMUDeque.size() == 100);

#if 1
    std::cout << "<<<<<<<<<<< test for imu data : \n " << std::endl;
    for(unsigned i = 0; i < testIMUDeque.size(); ++i) {
        IMUMeasure &imu = testIMUDeque.back();
        std::cout << "\timu data: " << i << std::endl
                << "\ttimeStamp: " << imu.timeStamp << std::endl
                << "\tacc: " << imu.measurement.acceleration << std::endl
                << "\tgyroscopes " << imu.measurement.gyroscopes << std::endl;
        testIMUDeque.pop_back();
    }
#endif // for python data test

}
