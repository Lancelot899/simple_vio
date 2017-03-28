#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>

#include <boost/regex.hpp>

#include "ImageIO.h"

ImageIO::ImageIO(std::string &imagefile)
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

    getline(imgData_file,totalLine);
    while (!imgData_file.eof()){
        if(totalLine[0]=='#') {
            getline(imgData_file,totalLine);
            continue;
        }

        std::size_t currentPos = -1;
        currentPos = totalLine.find(",",0);
        if(currentPos==std::string::npos){
            std::cout<<"No , !!!\n\n\n";
            break;
        }

//        timestamp = atoll(totalLine.substr(0,currentPos).c_str());
        fileName = totalLine.substr(currentPos+1,totalLine.size());

        imageDeque.push_back(fileName);
        getline(imgData_file,totalLine);
    };

//        timestampDeque.push_back(timeStamp);;
//        imageDeque.push_back(imageName);
//    };

}

std::string ImageIO::pop()
{
    std::string data;
    if(imageDeque.empty())
        return data;

    data = imageDeque.front();
    imageDeque.pop_front();
    return data;
}
