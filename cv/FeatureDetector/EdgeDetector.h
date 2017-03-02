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

        virtual ~EdgeDetector()
        {
            if(randomPattern != nullptr){
                delete[] randomPattern;
                randomPattern = nullptr;
            }
            if(gradHist != nullptr){
                delete[] gradHist;
                gradHist = nullptr;
            }
            if(thresholdSmoothed != nullptr){
                delete[] thresholdSmoothed;
                thresholdSmoothed = nullptr;
            }
            if(threshold != nullptr){
                delete[] threshold;
                threshold = nullptr;
            }
        }

        virtual void detect(
                cvframePtr_t frame,
                const ImgPyr_t &img_pyr,
                const double detection_threshold,
                features_t &fts);

    private:
        void makeHists(cvframePtr_t frame);

    private:
        int                  *gradHist;
        float                *threshold;
        double               *thresholdSmoothed;
        unsigned char        *randomPattern;
        int                  thresholdStepU, thresholdStepV;

        cvframePtr_t         currentFrame;
        features_t           edge;
    };

}

#endif //SIMPLE_VIO_EDGEDETECTOR_H
