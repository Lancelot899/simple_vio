//
// Created by lancelot on 2/19/17.
//

#include "DataStructure/cv/Feature.h"
#include "Detector.h"
#include "util/setting.h"

using namespace feature_detection;

Detector::Detector(const int img_width, const int img_height, const int cell_size, const int n_pyr_levels) {
    this->fastDetector = std::make_shared<FastDetector>(img_width, img_height, cell_size, n_pyr_levels);
    this->edgeDetector = std::make_shared<EdgeDetector>(img_width, img_height, cell_size, n_pyr_levels);
}

void Detector::detect(cvframePtr_t frame, const ImgPyr_t &img_pyr, features_t &fts) {
    fastDetector->detect(frame, img_pyr, fast_threshold, fts);
    edgeDetector->detect(frame, img_pyr, edge_threshold, fts);
}
