#include <stdio.h>
#include <fstream>
#include <iostream>

#include <boost/regex.hpp>

#include "ImageIO.h"
#include "util/util.h"


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
	isUndistortion = false;
}

ImageIO::ImageIO(std::string &imagefile,
                 std::string dataDirectory_,
                 std::shared_ptr<AbstractCamera> cam) : dataDirectory(dataDirectory_) {
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
	isUndistortion = true;
	cam_ = cam;
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

	if(!isUndistortion)
		return image;

	cv::Mat undistorionImage = Undistort(image, cam_);
    return undistorionImage;
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


	if(!isUndistortion)
		return std::make_pair(timeStamp, image);

	cv::Mat undistorionImage = Undistort(image, cam_);
    return std::make_pair(timeStamp,undistorionImage);
}
