//
// Created by lancelot on 1/5/17.
//

#ifndef SIMPLE_VIO_EDGEDETECTOR_H
#define SIMPLE_VIO_EDGEDETECTOR_H

#include "AbstractDetector.h"

namespace feature_detection {

    class EdgeDetector : public AbstractDetector {
    public:
        EdgeDetector(
                const int img_width,
                const int img_height,
                const int cell_size,
                const int n_pyr_levels);

        virtual ~EdgeDetector() {}

        virtual void detect(
                cvframePtr_t frame,
                const ImgPyr_t &img_pyr,
                const double detection_threshold,
                features_t &fts);


    private:


    };

}

#endif //SIMPLE_VIO_EDGEDETECTOR_H
