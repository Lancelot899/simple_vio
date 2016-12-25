#include <fstream>
#include <iostream>

#include "IMUIO.h"

IMUIO::IMUIO(std::__cxx11::string &file) {
    std::ifstream imu_file(file);
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

std::shared_ptr<IMUMeasure> IMUIO::pop(okvis::Time &start, okvis::Time &end) {

}
