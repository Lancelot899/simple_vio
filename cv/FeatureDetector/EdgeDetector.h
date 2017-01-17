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
                const int cell_size,  ///<! Todo
                const int n_pyr_levels);

        virtual ~EdgeDetector()
        {
            delete[] randomPattern;
            delete[] gradHist;
            delete[] thresholdSmoothed;
            delete[] threshold;
        }

        virtual void detect(
                cvframePtr_t frame,
                const ImgPyr_t &img_pyr,
                const double detection_threshold,
                features_t &fts);

    private:
        void makeHists(cvframePtr_t frame);
        int sample(cvframePtr_t frame,float *rate);

    private:
        int                  *gradHist;   ///<! ToDo
        float                *threshold;
        float                *thresholdSmoothed;
        unsigned char        *randomPattern;
        int                  thresholdStepU, thresholdStepV;

        cvframePtr_t         currentFrame;
        features_t           edge;  //std::list<std::shared_ptr<Feature>>
    };

}

#endif //SIMPLE_VIO_EDGEDETECTOR_H
