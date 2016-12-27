#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>

#include <boost/regex.hpp>

#include "IMUIO.h"

IMUIO::IMUIO(std::__cxx11::string &imufile, std::__cxx11::string &imuParamfile) {
    assert(!imufile.empty() && !imuParamfile.empty());

    std::ifstream imuParam_file(imuParamfile);
    if(!imuParam_file.good()) {
        std::cerr << "imuParam file error!" << std::endl;
        exit(-1);
    }

    char imuParamBuffer[1024];
    memset(imuParamBuffer, 0, 1024);

    imuParam_file.read(imuParamBuffer, 1024);
    boost::regex imuIDReg("^[A-Za-z]+\d+");
    boost::cmatch IDMat;
    if(!boost::regex_match(imuParamBuffer, IDMat, imuIDReg)) {
        std::cerr << "on imu ID in imu param !" << std::endl;
        exit(-1);
    }

    const std::string imuID = IDMat[0].str();
    char *p = strstr(imuParamBuffer, "T_BS");
    if(p == NULL) {
        std::cerr << "on T_BS in imu param !\n";
        exit(-1);
    }

    p = strstr(p, "data: [");
    p += strlen("data: [");
    double T_BS[16];
    memset(T_BS, 0, sizeof(double) * 16);
    int T_BSCnt = 0;

    char *p1 = p;
    while(*p1 != ']') p1++;
    *p1 = '\0';
    char *spt = strtok(p, ",");
    while(spt != NULL) {
        if((*spt >= '0' && *spt <='9') || (*spt = '.')) {
            sscanf(spt, "%lf", T_BS + T_BSCnt);
        }

        else {
            char *pspt = spt + 1;
            while(!((*pspt >= '0' && *pspt <='9') || (*pspt = '.'))) pspt++;
            sscanf(pspt, "%lf", T_BS + T_BSCnt);
        }

        T_BSCnt++;
        spt = strtok(NULL, ",");
    }

    int rate_hz = -1;
    p = p1 + 1;
    p = strstr(p, "rate_hz: ");
    if(p != NULL) {
        p += strlen("rate_hz: ");
        p1 = p;
        while(*p1 >= '0' && *p1 <= '9') p1++;
        char cp1 = *p1;
        *p1 = '\0';
        sscanf(p, "%d", rate_hz);
    }
    else
        p = p1 + 1;


    ///! <other


    std::ifstream imu_file(imufile);
    if(!imu_file.good()) {
        std::cerr << "imu file error!" << std::endl;
        exit(-1);
    }




    do {
        std::string line;
        if(!std::getline(imu_file, line)) {
            std::cout << "imu file is read over!" << std::endl;
            break;
        }

        std::stringstream stream(line);
        std::string s;
        std::getline(stream, s, ',');
        std::string nanoseconds = s.substr(s.size() - 9, 9);
        std::string seconds = s.substr(0, s.size() - 9);
        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            gyr[j] = std::stof(s);
        }

        Eigen::Vector3d acc;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            acc[j] = std::stof(s);
        }

        okvis::Time t_imu(std::stoi(seconds), std::stoi(nanoseconds));
        imuMeasureDeque.addImuMeasurement(0, t_imu, acc, gyr);
    } while(true);

}

IMUIO::dataDeque_t IMUIO::pop(okvis::Time &start, okvis::Time &end) {
    dataDeque_t imuDeque;
    std::shared_ptr<IMUMeasure>& pImuMeasure = imuMeasureDeque.front();
    if(pImuMeasure->measurement.timeStamp > end)
        return imuDeque;

    while(pImuMeasure->measurement.timeStamp < start) imuMeasureDeque.pop_front();
    auto it = std::find_if(imuMeasureDeque.begin(), imuMeasureDeque.end(), [&](std::shared_ptr<IMUMeasure>& ptr) {
            ptr->measurement.timeStamp > end;
            return true;
});

    if(it != imuMeasureDeque.end())
        it++;

    std::move(imuMeasureDeque.begin(), it, imuDeque.begin());
    return imuDeque;
}

IMUIO::pData_t IMUIO::pop() {
    if(imuMeasureDeque.empty())
        return pData_t();

    pData_t & data = imuMeasureDeque.front();
    imuMeasureDeque.pop_front();
    return data;
}

const IMUIO::pImuParam &IMUIO::getImuParam() {
    return imuParam;
}
