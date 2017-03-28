#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>

#include <boost/regex.hpp>

#include "ImageIO.h"

ImageIO::ImageIO(std::string &imagefile, std::string cameraParamFile, std::string dataDirectory_):dataDirectory(dataDirectory_)
{
    assert(!imagefile.empty());
    double timestamp = 0.0;
    std::string fileName;
    std::string totalLine;
    std::ifstream imgData_file(imagefile.c_str());
    if(!imgData_file.good()) {
        std::cerr << "camParam_file file error!" << std::endl;
        exit(-1);
    }

    std::getline( imgData_file, totalLine );
    while (!imgData_file.eof()){
        if(totalLine[0]=='#') {
            imgData_file>>totalLine;
            continue;
        }

        std::size_t currentPos = -1;
        currentPos = totalLine.find(",",0);
        if(currentPos==std::string::npos){
            std::cout<<"No , !!!\n\n\n";
            break;
        }
        timestamp = atoll(totalLine.substr(0,currentPos).c_str());
        fileName = totalLine.substr(currentPos+1,totalLine.size());
        imageDeque.push_back(std::make_pair(timestamp,fileName));

        imgData_file>>totalLine;
    }
    parseParamFile(cameraParamFile);

}

std::string ImageIO::popName()
{
    std::string data;
    if(imageDeque.empty())
        return data;

    data = imageDeque.front().second;
    imageDeque.pop_front();
    return data;
}

int ImageIO::parseParamFile(std::string ParamFile)
{
    std::ifstream camParam_file(ParamFile.c_str());
    if(!camParam_file.good()) {
        std::cerr << "camParam_file file error!" << std::endl;
        exit(-1);
    }

    char camParamBuffer[1024];
    memset(camParamBuffer, 0, 1024);
    camParam_file.read(camParamBuffer, 1024);
    assert(camParamBuffer[0] != 0);

    char *p = camParamBuffer, *pend = camParamBuffer;

    ///camera_model
    p = strstr(camParamBuffer, "camera_model: ");
    p += strlen("camera_model: ");
    pend = p; pend = strstr(pend,"\n");
    char camera_model[pend-p+1]; memset(camera_model,0,pend-p+1); memcpy(camera_model,p,pend-p);
    std::string Camera_mode(camera_model);

    ///distortion_model
    p = strstr(camParamBuffer, "distortion_model: ");
    p += strlen("distortion_model: ");
    pend = p; pend = strstr(pend,"\n");
    char distortion_model[pend-p+1];  memset(distortion_model,0,pend-p+1); memcpy(distortion_model,p,pend-p);
    std::string Distortion_mode(distortion_model);

    ///intrinsics
    p = strstr(camParamBuffer, "intrinsics: [");
    p += strlen("intrinsics: ["); pend=p;

    for (int i = 0; i < 4; ++i) {
        while( *p != '.' && *p != '-' && *p!= 'e' && ( *p > '9' | *p < '0') ) p++;
        if(i==3)
            pend = strstr(p , "]");
        else
            pend = strstr(p , ",");

        char valueIntri0[pend-p+1];
        memset(valueIntri0,0,pend-p+1);
        memcpy(valueIntri0,p,pend-p);
        p = pend;

        intrinsics[i] = atof(valueIntri0);
    }

    ///distortion_coefficients
    p = strstr(camParamBuffer, "distortion_coefficients: [");
    p += strlen("distortion_coefficients: ["); pend=p;

    for (int i = 0; i < 4; ++i) {
        while( *p != '.' && *p != '-' && *p!= 'e' && ( *p > '9' | *p < '0') ) p++;
        if(i==3)
            pend = strstr(p , "]");
        else
            pend = strstr(p , ",");

        char valueDistor0[pend-p+1];
        memset(valueDistor0,0,pend-p+1);
        memcpy(valueDistor0,p,pend-p);
        p = pend;

        distortion_coefficients[i] = atof(valueDistor0);
    }
}

cv::Mat ImageIO::popImage()
{
    std::string data;
    if(imageDeque.empty())
        return cv::Mat();

    data = imageDeque.front().second;
    imageDeque.pop_front();
    data = dataDirectory + data;
    cv::Mat image = cv::imread(data,0);
    if(image.empty()) {
        std::cout<<data<<"!\n";
        return cv::Mat();
    }

    static cv::Mat K_ = (cv::Mat_<double>(3, 3) << intrinsics[0], 0.0, intrinsics[2], 0.0, intrinsics[1], intrinsics[3], 0.0, 0.0, 1.0);
    static cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << distortion_coefficients[0],distortion_coefficients[1],distortion_coefficients[2],distortion_coefficients[3]);
    cv::Mat tmp_;
    cv::undistort(image, tmp_, K_, distCoeffs);
    return tmp_;
}

std::pair<double, cv::Mat> ImageIO::popImageAndTimestamp()
{
    std::string data;
    if(imageDeque.empty())
        return std::pair<double, cv::Mat>();

    double timeStamp = imageDeque.front().first;
    data = imageDeque.front().second;
    imageDeque.pop_front();
    data = dataDirectory + data;
    cv::Mat image = cv::imread(data,0);
    if(image.empty()) {
        std::cout<<data<<" is empty!\n";
        return std::pair<double, cv::Mat>();
    }

    static cv::Mat K_ = (cv::Mat_<double>(3, 3) << intrinsics[0], 0.0, intrinsics[2], 0.0, intrinsics[1], intrinsics[3], 0.0, 0.0, 1.0);
    static cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << distortion_coefficients[0],distortion_coefficients[1],distortion_coefficients[2],distortion_coefficients[3]);
    cv::Mat tmp_;
    cv::undistort(image, tmp_, K_, distCoeffs);

    return std::make_pair(timeStamp,tmp_);
}
