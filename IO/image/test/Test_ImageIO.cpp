#include <opencv2/ts/ts.hpp>
#include <opencv2/opencv.hpp>
#include "../ImageIO.h"


TEST(TESTIMAGEIO,TESTIMAGEIO){
    std::cout<<"Enter TestImageIO"<<std::endl;
    std::string imageFile = "../testData/mav0/cam0/data.csv";
    std::string camParamfile ="../testData/mav0/cam0/sensor.yaml";
    ImageIO imageTest(imageFile,camParamfile,"../testData/mav0/cam0/data/");

    std::pair<double, cv::Mat> data;
    char key = 0;
    while (key != 27) {
        data = imageTest.popImageAndTimestamp();
        cv::Mat image = data.second;
        if(!image.empty()){
            cv::imshow("image",image);
            key = cv::waitKey();
        }
    }
    std::cout<<"End of TestImageIO\n";

}
