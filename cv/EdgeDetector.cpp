//
// Created by lancelot on 1/5/17.
//

#include "EdgeDetector.h"
#include "DataStructure/cv/Feature.h"

namespace feature_detection {

    using namespace std;
    using namespace Eigen;

    EdgeDetector::EdgeDetector(
            const int img_width,
            const int img_height,
            const int cell_size,
            const int n_pyr_levels) :
            AbstractDetector(img_width, img_height, cell_size, n_pyr_levels) {}

    void EdgeDetector::detect(cvframePtr_t frame,
                              const AbstractDetector::ImgPyr_t &img_pyr,
                              const double detection_threshold,
                              AbstractDetector::features_t &fts) {


    }

    class Pt {
    public:
        float grad;
        Vector2d xy;

        Pt() {
            xy[0] = 0;
            xy[1] = 0;
            grad = 0.0;
        }

        Pt(float grad_, Vector2d xy_) : grad(grad_), xy(xy_) {}

        bool operator<(const Pt &m) const { return grad > m.grad; }
    };

    class Edgelete {
    public:
        float grad;
        Vector2d xy;
        Vector2d dir;

        Edgelete() {
            xy[0] = 0;
            xy[1] = 0;
            grad = 0.0;
        }

        Edgelete(float grad_, Vector2d xy_, Vector2d dir_) : grad(grad_), xy(xy_), dir(dir_) {}

        bool operator<(const Edgelete &m) const { return grad > m.grad; }
    };
}