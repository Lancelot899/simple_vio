//
// Created by lancelot on 1/5/17.
//

#ifndef SIMPLE_VIO_FASTDETECTOR_H
#define SIMPLE_VIO_FASTDETECTOR_H

#include "AbstractDetector.h"

namespace feature_detection {
    /// FAST detector by Edward Rosten.
    class FastDetector : public AbstractDetector {
        friend class Detector;
    public:
        FastDetector(
                const int img_width,
                const int img_height,
                const int cell_size,
                const int n_pyr_levels);

        virtual ~FastDetector() {}

        virtual void detect(
                cvframePtr_t frame,
                const ImgPyr_t& img_pyr,
                const double detection_threshold,
                features_t& fts);

    protected:
        struct fast_xy {
            short x, y;
            fast_xy(short x_, short y_) : x(x_), y(y_) {}
        };

        void fast_corner_detect_10(const cvData::Img_t& img, int imgWidth, int imgHeight,
                                   int img_stride, double barrier, std::vector<fast_xy>& corners);

        void fast_corner_score_10(cvData::Img_t::const_iterator img, const int img_stride,
                                  const std::vector<fast_xy>& corners, const double threshold, std::vector<double>& scores);

        void fast_nonmax_3x3(const std::vector<fast_xy>& corners,
                             const std::vector<double>& scores,
                             std::vector<double>& nonmax_corners);
    };

}

#endif //SIMPLE_VIO_FASTDETECTOR_H
