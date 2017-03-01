//
// Created by lancelot on 2/27/17.
//

#include "../Tracker.h"
#include "opencv2/ts/ts.hpp"
#include "DataStructure/viFrame.h"
#include "ThirdParty/sophus/se3.hpp"
#include "DataStructure/cv/cvFrame.h"

TEST(Tracker, Tracker) {
//    direct_tracker::Tracker tracker;
//    std::shared_ptr<cvFrame> cvframe_i = std::make_shared<cvFrame>(cam, pic_i);
//    std::shared_ptr<viFrame> viframe_i = std::make_shared<viFrame>(1, cvframe_i);
//
//    std::shared_ptr<cvFrame> cvframe_j = std::make_shared<cvFrame>(cam, pic_j);
//    std::shared_ptr<viFrame> viframe_j = std::make_shared<viFrame>(2, cvframe_j);
//
//    Sophus::SE3d se3;
//
//    //! test runing time
//    bool isTracked = tracker.Tracking(viframe_i, viframe_j, se3, 50);
//
//
//    if(isTracked)
//        printf("successful!\n");
//    else
//        printf("failed!\n");
}
