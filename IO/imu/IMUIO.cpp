#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>

#include <boost/regex.hpp>

#include "IMUIO.h"

#include "util/util.h"

void inline readData(char *buffer, const char *name, double *val) {
    char *p = buffer;
    p = strstr(p, name);

    assert(p != NULL);
    *val = -0.1;
    p += strlen(name);
    char *p1 = p;
    while((*p1 >= '0' && *p1 <= '9') || (*p1 == '.') || (*p1 == 'e') || (*p1 == '-') || (*p1 == '+')) p1++;
    char c = *p1;
    *p1 = 0;
    sscanf(p, "%le", val);
    *p1 = c;
}


IMUIO::IMUIO(std::string &imufile, std::string &imuParamfile) {
    assert(!imufile.empty() && !imuParamfile.empty());
    std::ifstream imuParam_file(imuParamfile);
    if(!imuParam_file.good()) {
        std::cerr << "imuParam file error!" << std::endl;
        exit(-1);
    }

    char imuParamBuffer[1024];
    memset(imuParamBuffer, 0, 1024);

    imuParam_file.read(imuParamBuffer, 1024);

    assert(imuParamBuffer[0] != 0);
//    boost::regex imuIDReg("^[A-Za-z]+[0-9]+");
//    boost::cmatch IDMat;
//    if(!boost::regex_match(imuParamBuffer, IDMat, imuIDReg)) {
//        std::cerr << "no imu ID in imu param !" << std::endl;
//        exit(-1);
//    }
//
//    const std::string imuID = IDMat[0].str();

    char c = 0;
    int rate_hz = -1;
    char *p = imuParamBuffer;
    p = strstr(p, "rate_hz: ");
    assert(p != NULL);
    p += strlen("rate_hz: ");
    char * p1 = p;
    while (*p1 >= '0' && *p1 <= '9') p1++;
    c = *p1;
    *p1 = '\0';
    sscanf(p, "%d", &rate_hz);
    *p1 = c;

    double gyroscope_noise_density = -0.1;
    readData(imuParamBuffer, "gyroscope_noise_density: ", &gyroscope_noise_density);

    double gyroscope_random_walk = -0.1;
    readData(imuParamBuffer, "gyroscope_random_walk: ", &gyroscope_random_walk);

    double accelerometer_noise_density = -0.1;
    readData(imuParamBuffer, "accelerometer_noise_density: ", &accelerometer_noise_density);

    double accelerometer_random_walk = -0.1;
    readData(imuParamBuffer, "accelerometer_random_walk: ", &accelerometer_random_walk);

    p = strstr(imuParamBuffer, "T_BS");
    if(p == NULL) {
        std::cerr << "on T_BS in imu param !\n";
        exit(-1);
    }

    p = strstr(p, "data: [");
    p += strlen("data: [");
    double T_BS[16];
    memset(T_BS, 0, sizeof(double) * 16);
    int T_BSCnt = 0;

    p1 = p;

    while(*p1 != ']') p1++;
    c = *p1;
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

    *p1 = c;

    imuParam = std::make_shared<ImuParameters>();
    imuParam->rate = rate_hz;
    imuParam->sigma_bg  = gyroscope_random_walk;
    imuParam->sigma_ba  = accelerometer_random_walk;
    imuParam->sigma_a_c = accelerometer_noise_density;
    imuParam->sigma_g_c = gyroscope_noise_density;

    Eigen::Matrix4d mTbs;
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            mTbs(i, j) = T_BS[j * 4 + i];
        }
    }

    Sophus::SE3d seTbs(mTbs);

    imuParam->T_BS = seTbs;

    std::ifstream imu_file(imufile);
    if(!imu_file.good()) {
        std::cerr << "imu file error!" << std::endl;
        exit(-1);
    }

    int i = -1;
    do {
        i++;
        std::string line;
        if(!std::getline(imu_file, line)) {
            break;
        }

        if(i == 0)
            continue;

        std::stringstream stream(line);
        std::string s;
        std::getline(stream, s, ',');
        std::string nanoseconds = s.substr(s.size() - 9, 9);
        std::string seconds = s.substr(0, s.size() - 9);
        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            sscanf(s.c_str(), "%lf", &gyr[j]);
            //gyr[j] = std::stod(s);
        }


        Eigen::Vector3d acc;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            sscanf(s.c_str(), "%lf", &acc[j]);
            //acc[j] = std::stod(s);
        }

        int sec = 0;
        int nsec = 0;
        sscanf(seconds.c_str(), "%d", &sec);
        sscanf(nanoseconds.c_str(), "%d", &nsec);
        okvis::Time t_imu(sec, nsec);
        imuMeasureDeque.addImuMeasurement(0, t_imu, acc, gyr);
    } while(true);

}

IMUIO::dataDeque_t IMUIO::pop(okvis::Time &start, okvis::Time &end) {
    dataDeque_t imuDeque;
    std::shared_ptr<IMUMeasure> pImuMeasure = imuMeasureDeque.front();
    if(pImuMeasure->measurement.timeStamp > end)
        return imuDeque;

    while(pImuMeasure->measurement.timeStamp < start){
        imuMeasureDeque.pop_front();
        pImuMeasure = imuMeasureDeque.front();
    }

    auto it = std::find_if(imuMeasureDeque.begin(), imuMeasureDeque.end(), [&](std::shared_ptr<IMUMeasure>& ptr) {
            if(ptr->measurement.timeStamp >= end)
                return true;
	        return false;
});

    if(it != imuMeasureDeque.end())
        it++;

    int cnt= 0;
    for(auto it_ = imuMeasureDeque.begin(); it_ != it; ++it_) {
        imuDeque.push_back(*it_);
    }

    while(cnt) {
        imuMeasureDeque.pop_front();
        cnt--;
    }

    return imuDeque;
}

IMUIO::pData_t IMUIO::pop() {
    if(imuMeasureDeque.empty())
        return pData_t();

    pData_t data;
    data = std::move(imuMeasureDeque.front());
    imuMeasureDeque.pop_front();
    return data;
}

const IMUIO::pImuParam &IMUIO::getImuParam() {
    return imuParam;
}
