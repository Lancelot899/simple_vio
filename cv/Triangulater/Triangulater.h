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

    bool triangulate(std::shared_ptr<viFrame>&keyFrame, std::shared_ptr<viFrame>&nextFrame, Sophus::SE3d T_nk);
};



#endif
