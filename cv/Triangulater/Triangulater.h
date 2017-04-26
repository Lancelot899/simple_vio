#ifndef SIMPLE_VIO_TRIANGULATER
#define SIMPLE_VIO_TRIANGULATER

#include <memory>
#include "ThirdParty/sophus/se3.hpp"

class cvFrame;
class viFrame;

#define epilolineThreshold 10

class Triangulater
{
public:
    Triangulater();
    ~Triangulater(){}

    int triangulate(std::shared_ptr<viFrame>&keyFrame,
                    std::shared_ptr<viFrame>&nextFrame, const Sophus::SE3d &T_nk,
                    Eigen::Matrix<double, 6, 6>&infomation,int iter = 0);
};



#endif
