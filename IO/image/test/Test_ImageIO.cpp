#include <opencv2/ts/ts.hpp>
#include <opencv2/opencv.hpp>
#include "../ImageIO.h"


TEST(TESTIMAGEIO,TESTIMAGEIO){
    std::cout<<"Enter TestImageIO"<<std::endl;
    std::string imageFile = "../testData/mav0/cam0/data.csv";
    ImageIO imageTest(imageFile,"../testData/mav0/cam0/data/");

    cv::Mat image = imageTest.popImage();
    char key = 0;
    std::cout<<"TestImageIO start while\n";
    while (key != 27) {
        if(!image.empty()){
            cv::imshow("image",image);
            key = cv::waitKey();
        }
        image = imageTest.popImage();
    }
    std::cout<<"End of TestImageIO\n";

}
