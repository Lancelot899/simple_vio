#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>

#include <boost/regex.hpp>

#include "ImageIO.h"

ImageIO::ImageIO(std::string &imagefile, std::string dataDirectory_):dataDirectory(dataDirectory_)
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

    imgData_file>>totalLine;
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

        fileName = totalLine.substr(currentPos+1,totalLine.size());
        imageDeque.push_back(fileName);

        imgData_file>>totalLine;
    }

}

std::string ImageIO::popName()
{
    std::string data;
    if(imageDeque.empty())
        return data;

    data = imageDeque.front();
    imageDeque.pop_front();
    return data;
}

cv::Mat ImageIO::popImage()
{
    std::string data;
    if(imageDeque.empty())
        return cv::Mat();

    data = imageDeque.front();
    imageDeque.pop_front();
    data = dataDirectory + data;
    cv::Mat image = cv::imread(data,0);
    if(image.empty()) std::cout<<data<<"!\n";
    return image;
}
