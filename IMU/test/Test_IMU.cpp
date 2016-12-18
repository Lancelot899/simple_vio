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
    IMUTest = PyImport_ImportModule("IMUTest");
    ASSERT_TRUE(IMUTest != NULL);
    getData = PyObject_GetAttrString(IMUTest, "getData");
    ASSERT_TRUE(getData != NULL);
    PyObject *getDataArgs = PyTuple_New(2);
    PyTuple_SetItem(getDataArgs, 0, Py_BuildValue("s", "data.csv"));
    PyTuple_SetItem(getDataArgs, 1, Py_BuildValue("i", 100));     ///< get 100 data
    PyObject *pyDatas = NULL;
    pyDatas = PyEval_CallObject(getData, getDataArgs);
    ASSERT_TRUE(pyDatas!= NULL);
    int dataSize = PyList_Size(pyDatas);
    ASSERT_TRUE(dataSize == 100);
    printf("data size = 100, read file over!\n");
    ImuMeasureDeque testIMUDeque;
    PyObject* pyData = NULL;
    PyObject* pyItem = NULL;
    for(int i = 0; i < dataSize; ++i) {
        pyData = PyList_GetItem(pyDatas, i);
        ASSERT_TRUE(pyData != NULL);
        ASSERT_TRUE(PyList_Size(pyData) == 7);
        double pyArg[7];
        memset(pyArg, 0, sizeof(double) * 7);
        for(int j = 0; j < 7; ++j) {
            pyItem = PyList_GetItem(pyData, j);
            ASSERT_TRUE(pyItem != NULL);
            PyArg_Parse(pyItem, "d", pyArg + j);
            pyItem = NULL;
        }
        pyData = NULL;
        double timeStamp = pyArg[0];
        Eigen::Vector3d gyroscopes(pyArg[1],pyArg[2],pyArg[3]);
        Eigen::Vector3d acceleration(pyArg[4], pyArg[5], pyArg[6]);
        testIMUDeque.push_back(IMUMeasure(0, timeStamp, acceleration, gyroscopes));
    }

    ImuParamenters imuParam;
    imuParam.a_max = 1;
    imuParam.g_max = 1;


    Py_DECREF(getDataArgs);
    Py_DECREF(IMUTest);
    Py_Finalize();

    ///< over!
    ASSERT_TRUE(testIMUDeque.size() == 100);

#if 0
    std::cout << "<<<<<<<<<<< test for imu data : \n " << std::endl;
    for(unsigned i = 0; i < testIMUDeque.size(); ++i) {
        IMUMeasure &imu = testIMUDeque.back();
        std::cout << "imu data: " << i << std::endl
                  << "timeStamp: " << std::endl << imu.timeStamp << std::endl
                  << "acc: " << std::endl << imu.measurement.acceleration << std::endl
                  << "gyroscopes: " << std::endl << imu.measurement.gyroscopes << std::endl;
        printf("\n");
        testIMUDeque.pop_back();
    }
    std::cout << "<<<<<<<<<<<< get data right!\n" << std::endl;
#endif // for python data test

    ///< test okvis integration
    IMU imu(IMU::IntegalType::OKVIS_INTEGRATION);

}
