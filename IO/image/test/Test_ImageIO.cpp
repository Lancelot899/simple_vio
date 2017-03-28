#include <opencv2/ts/ts.hpp>
#include <opencv2/opencv.hpp>
#include "../ImageIO.h"


TEST(TESTIMAGEIO,TESTIMAGEIO){
    std::cout<<"Enter TestImageIO"<<std::endl;
    std::string imageFile = "../testData/mav0/cam0/data.csv";
    ImageIO imageTest(imageFile);

    std::string fileName_ = imageTest.pop();
    char key = 0;
    std::cout<<"TestImageIO start while\n";
    while (fileName_.size() && key != 27) {
        fileName_ = "../testData/mav0/cam0/data/" + fileName_;

        cv::Mat image = cv::imread(fileName_,0);
        std::cout<<"Name_ "<<fileName_<<"\n width = "<<image.cols<<", height = "<<image.rows<<"\n\n";
        if(!image.empty()){
            cv::imshow(fileName_,image);
            key = cv::waitKey();
        }
        fileName_ = imageTest.pop();
    }
    std::cout<<"End of TestImageIO\n";

}
