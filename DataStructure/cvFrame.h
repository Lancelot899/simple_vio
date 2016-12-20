#ifndef viFrame_H
#define viFrame_H

#include <sophus/se3.hpp>

typedef Sophus::SE3d pose_t;

class cvFrame
{
public:
    cvFrame();
    ~cvFrame();

    void setPose(pose_t pose) {
        this->pose = pose;
    }

    const pose_t getPose() {
        return pose;
    }

private:
    int    id;
    pose_t pose;
};

#endif // viFrame_H
