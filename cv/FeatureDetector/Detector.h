//
// Created by lancelot on 2/19/17.
//

#ifndef SIMPLE_VIO_DETECTOR_H
#define SIMPLE_VIO_DETECTOR_H

#include <memory>

#include "EdgeDetector.h"
#include "FastDetector.h"

namespace feature_detection {
    class Detector {
    public:
        Detector(
                const int img_width,
                const int img_height,
                const int cell_size,  ///<! Todo
                const int n_pyr_levels);

        void detect(
                cvframePtr_t frame,
                const ImgPyr_t &img_pyr,
                features_t &fts);
    private:
        std::shared_ptr<FastDetector> fastDetector;
        std::shared_ptr<EdgeDetector> edgeDetector;
    };

}

#endif //SIMPLE_VIO_DETECTOR_H
